#include "KMDFQPU.h"
#include "NTMATHS.h"

// WoR-Project 2021
// KMDFQPU - The Initial Port of "hello_fft" by Andrew Holme 2015 from the Raspberry Pi Userland.
// Written for Raspberry Pi 4 Model B

//////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region // DEFINES

#define GPU_FFT_QPUS   8

#define GPU_FFT_FWD   0 // Forward FFT
#define GPU_FFT_REV   1 // Inverse FFT

#define GPU_FFT_NO_FLUSH   1
#define GPU_FFT_ALIGN   4096 // Bytes

#define GPU_FFT_BUSY_WAIT_LIMIT   (5 << 12) // ~1ms
#define VC_TRANSLATE(address)     ((address) & 0x3FFFFFFF)

#define GPU_FFT_PI   3.14159265358979323846

#define ALPHA(dx)   (2 * pow(sin((dx) / 2), 2))
#define BETA(dx)    (sin(dx))

static double k[16] = { 0, 8, 4, 4, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1 };
static double m[16] = { 0, 0, 0, 1, 0, 1, 2, 3, 0, 1, 2, 3, 4, 5, 6, 7 };

static PULONG V3DHandle = NULL;
static PULONG PowerHandle = NULL;
static PULONG MailboxHandle = NULL;

static PULONG ASBHandle = NULL;

__declspec(align(16)) volatile PULONG MailboxPacket; // 0xF reserved for 4-bit channel
static PMDL MailboxMiddle = NULL;

volatile int t1 = 0;
volatile int t2 = 0;
volatile int t3 = 0;

#define PI_PERI_BASE                         0xFE000000   // Low-Peripheral Mode
#define PI_PERI_LENGTH                        0x1000000

#define MBOX_BASE                (PI_PERI_BASE + 0xB880)  // MailboxHandle 0x0
#define MBOX_LENGTH                                0x48

#define MBOX_OFFSET_READ                              0   // 0x00 + 0xB880 + 0xFE000000
#define MBOX_OFFSET_STATUS                            6   // 0x18 + 0xB880 + 0xFE000000
#define MBOX_OFFSET_CONFIG                            7   // 0x1C + 0xB880 + 0xFE000000
#define MBOX_OFFSET_WRITE                             8   // 0x20 + 0xB880 + 0xFE000000

#define MBOX_MIN                             0x00000000   // Machine Memory Starting Offset
#define MBOX_MAX                             0x30000000   // 1GB - gpu_mem Address Limit
#define MBOX_MTU                                   1500   // Maximum Transmission Unit
#define MBOX_RETRIES                                100   // times
#define MBOX_TIMEOUT                                100   // us

#define MBOX_REQUEST                         0x00000000
#define MBOX_EMPTY                           0x40000000
#define MBOX_FULL                            0x80000000
#define MBOX_FAILURE                         0x80000001
#define MBOX_SUCCESS                         0x80000000

#define MBOX_TAG_ALLOCATE_MEMORY                0x3000C
#define MBOX_TAG_LOCK_MEMORY                    0x3000D
#define MBOX_TAG_UNLOCK_MEMORY                  0x3000E
#define MBOX_TAG_RELEASE_MEMORY                 0x3000F

#define V3D_BASE               (PI_PERI_BASE + 0xC04000)   // V3DHandle 0x0
#define V3D_LENGTH                                0xF24

#define V3D_OFFSET_IDENT0                             0   // 0x0000 + 0xC04000 + 0xFE000000
#define V3D_4D3V                             0x04443356   // ASCII 4"D3V"

#define V3D_OFFSET_L2CACTL                            8   // 0x0020 + 0xC04000 + 0xFE000000
#define V3D_OFFSET_SLCACTL                            9   // 0x0024 + 0xC04000 + 0xFE000000
#define V3D_OFFSET_SRQPC                            268   // 0x0430 + 0xC04000 + 0xFE000000
#define V3D_OFFSET_SRQUA                            269   // 0x0434 + 0xC04000 + 0xFE000000
#define V3D_OFFSET_SRQCS                            271   // 0x043C + 0xC04000 + 0xFE000000
#define V3D_OFFSET_DBCFG                            896   // 0x0E00 + 0xC04000 + 0xFE000000
#define V3D_OFFSET_DBQITE                           907   // 0x0E2C + 0xC04000 + 0xFE000000
#define V3D_OFFSET_DBQITC                           908   // 0x0E30 + 0xC04000 + 0xFE000000

#define V3D_POWER_BASE         (PI_PERI_BASE + 0x10010C)  // PowerHandle 0x0
#define V3D_POWER_LENGTH                            0x4

#define V3D_POWER_OFFSET_POWER                        0   // 0x0 + 0x10010C + 0xFE000000

#define V3D_ASB_BASE           (PI_PERI_BASE + 0xC11008)  // ASBHandle 0x0
#define V3D_ASB_LENGTH                              0x8

#define V3D_ASB_OFFSET_ASB_M                          1   // 0x4 + 0xC11008 + 0xFE000000
#define V3D_ASB_OFFSET_ASB_S                          0   // 0x0 + 0xC11008 + 0xFE000000
#define V3D_MAGIC                            0x5A000000   // ASCII "Z..."

#pragma endregion
//////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region // STRUCTS

typedef struct _GPU_FFT_COMPLEX
{
    float real, imaginary;
}   GPU_FFT_COMPLEX, *PGPU_FFT_COMPLEX;

typedef struct _GPU_FFT_BASE
{
    unsigned int shared_handle, shared_size, vc_msg, vc_code, vc_uniforms[GPU_FFT_QPUS];
}   GPU_FFT_BASE, *PGPU_FFT_BASE;

typedef struct _GPU_FFT_SHARE
{
    unsigned int vc;
    union
    {
        PGPU_FFT_COMPLEX cptr;
        void* vptr;
        char* bptr;
        float* fptr;
        unsigned int* uptr;
    };
}   GPU_FFT_SHARE, *PGPU_FFT_SHARE;

typedef struct _GPU_FFT
{
    GPU_FFT_BASE base;
    GPU_FFT_SHARE share;
    PGPU_FFT_COMPLEX input, output;
    int x, y, step;
}   GPU_FFT, *PGPU_FFT;

typedef struct _SHADERS
{
    unsigned int size, *code;
}   SHADERS, *PSHADERS;

typedef struct _TWIDDLES
{
    int passes, shared, unique;
    void (*twiddles) (double, float*);
}   TWIDDLES, *PTWIDDLES;

#pragma endregion
//////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region // FORWARDS

#define       mmio_read(Base, Offset)              READ_REGISTER_ULONG((PULONG)(Base + Offset))
#define       mmio_write(Base, Offset, Value)      WRITE_REGISTER_ULONG((PULONG)(Base + Offset), Value)

void          v3d_setup(unsigned int Enable);

unsigned int  mailbox_setup(unsigned char Channel);
#define       mailbox_peek()                       mmio_read(MailboxHandle,  MBOX_OFFSET_STATUS)
#define       mailbox_read()                       mmio_read(MailboxHandle,  MBOX_OFFSET_READ)
#define       mailbox_write(AddreCh)               mmio_write(MailboxHandle, MBOX_OFFSET_WRITE, AddreCh)

unsigned int  memory_allocate(unsigned int Size, unsigned int Align, unsigned int Flags);
unsigned int  memory_reserve(unsigned int SharedHandle, unsigned int Lock);
unsigned int  memory_free(unsigned int SharedHandle);

int           gpu_fft_prepare(GPU_FFT& Handle, int Log2_N, int Direction, int Jobs);
int           gpu_fft_execute(GPU_FFT& Handle);

int           gpu_fft_allocate(GPU_FFT& Handle, unsigned int Size);
int           gpu_fft_increment(GPU_FFT& Handle, int Bytes);
void          gpu_fft_release(GPU_FFT& Handle);

unsigned int  gpu_fft_shader_size(int Log2_N);
unsigned int* gpu_fft_shader_code(int Log2_N);

int           gpu_fft_twiddle_size(int Log2_N, int* Shared, int* Unique, int* Passes);
void          gpu_fft_twiddle_data(int Log2_N, int Direction, float* Output);

#pragma endregion
//////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region // MAIN

void main(int argc, char* argv[])
{
    debug("[CALL]: main");
    int result = 0;
    int loops = 0;
    int freq = 0;
    int log2_N = 0;
    int jobs = 0;
    int N = 0;

    GPU_FFT fft;
    double tsq[2];
    // unsigned t[2];
    PGPU_FFT_COMPLEX complex;

    log2_N = (argc > 1) ? atoi(argv[1]) : 12; // 8 <= log2_N <= 22
    jobs   = (argc > 2) ? atoi(argv[2]) : 1;  // Transforms Per Batch
    loops  = (argc > 3) ? atoi(argv[3]) : 1;  // Test Repetitions

    if ((argc < 2) || (jobs < 1) || (loops < 1))
    {
        debug("[ASRT]: (argc < 2) || (jobs < 1) || (loops < 1)");
        return;
    }

    N = 1 << log2_N; // FFT Length
    result = gpu_fft_prepare(fft, log2_N, GPU_FFT_REV, jobs); // Call Once

    switch (result)
    {
        case (-1): { debug("[ASRT]: V3D is Offline");    return; } break;
        case (-2): { debug("[ASRT]: 8 <= log2_N <= 22"); return; } break;
        case (-3): { debug("[ASRT]: Out of Memory");     return; } break;
        case (-4): { debug("[ASRT]: MMIO Failure");      return; } break;
        case (-5): { debug("[ASRT]: Platform Failure");  return; } break;
        default: break;
    }

    for (int l = 0; l < loops; ++l)
    {
        for (int j = 0; j < jobs; ++j)
        {
            complex = fft.input + (j * fft.step); // Input Buffer
            for (int i = 0; i < N; ++i) { complex[i].real = complex[i].imaginary = 0; }
            freq = (j + 1) & ((N / 2) - 1);
            complex[freq].real += complex[(N - freq) & (N - 1)].real = 0.5;
        }

        debug("[TEST]: Pre-Execution Sector Reached");
        gpu_fft_execute(fft); // Call one or many times
        debug("[TEST]: Post-Execution Sector Reached");

        tsq[0] = tsq[1] = 0;
        for (int j = 0; j < jobs; ++j)
        {
            complex = fft.output + ((long long)j * fft.step); // Output Buffer
            freq = (j + 1) & ((N / 2) - 1);
            for (int i = 0; i < N; ++i)
            {
                double real = cos((2 * GPU_FFT_PI * freq * i) / N);
                tsq[0] += pow(real, 2);
                tsq[1] += pow(real - complex[i].real, 2) + pow(complex[i].imaginary, 2);
            }
        }
        // printf("rel_rms_err = %0.2g, usecs = %d, k = %d\n",
        //     sqrt(tsq[1] / tsq[0]), (t[1] - t[0]) / jobs, k);
        double rel_rms_err = sqrt(tsq[1] / tsq[0]);
        debug("[TEST]: rel_rms_err = %0.2g", rel_rms_err);
    }
    gpu_fft_release(fft); // Videocore Memory lost if not freed!
    debug("[TEST]: Complete!");
}

#pragma endregion
//////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region // HARDWARE

void v3d_setup(unsigned int Enable)
{
    debug("[4D3V]: Powering On...");
    if (mmio_read(V3DHandle, V3D_OFFSET_IDENT0) == V3D_4D3V) { debug("[4D3V]: Previous State: Power On"); }
    else { debug("[4D3V]: Previous State: Power Off"); }

    if (Enable)
    {
        mmio_write(PowerHandle, V3D_POWER_OFFSET_POWER,  mmio_read(PowerHandle, V3D_POWER_OFFSET_POWER) | V3D_MAGIC  | 0x40);
        mmio_write(ASBHandle,   V3D_ASB_OFFSET_ASB_M,   (mmio_read(ASBHandle,   V3D_ASB_OFFSET_ASB_M)   | V3D_MAGIC) & 0xFFFFFFFE);
        mmio_write(ASBHandle,   V3D_ASB_OFFSET_ASB_S,   (mmio_read(ASBHandle,   V3D_ASB_OFFSET_ASB_S)   | V3D_MAGIC) & 0xFFFFFFFE);
    }
    else
    {
        mmio_write(PowerHandle, V3D_POWER_OFFSET_POWER,  mmio_read(PowerHandle, V3D_POWER_OFFSET_POWER) | V3D_MAGIC);
        mmio_write(ASBHandle,   V3D_ASB_OFFSET_ASB_M,   (mmio_read(ASBHandle,   V3D_ASB_OFFSET_ASB_M)   | V3D_MAGIC) | 0x1);
        mmio_write(ASBHandle,   V3D_ASB_OFFSET_ASB_S,   (mmio_read(ASBHandle,   V3D_ASB_OFFSET_ASB_S)   | V3D_MAGIC) | 0x1);
    }

    if (mmio_read(V3DHandle, V3D_OFFSET_IDENT0) == V3D_4D3V) { debug("[4D3V]: Current State: Power On"); }
    else { debug("[4D3V]: Current State: Power Off"); }
}

unsigned int mailbox_setup(unsigned char Channel)
{
    if ((!MailboxHandle) || (!MailboxPacket)) { debug("[MBOX]: Lock-up in Sector 7G, Sir"); return MBOX_FAILURE; }

    unsigned int checked = 0;
    unsigned int mail = ((MmGetPhysicalAddress(MailboxPacket).QuadPart) & ~0xF) | (Channel & 0xF); // 0xF Reserved for 4-bit Channel

    debug("[MBOX]: physical = 0x%016llX", MmGetPhysicalAddress(MailboxPacket).QuadPart);
    debug("[MBOX]: mail = 0x%08lX", mail);

    t1 = 0;
    t2 = 0;
    t3 = 0;

    for (int i = 0; i <= MBOX_OFFSET_CONFIG; ++i)
    {
        debug("[MBOX]: MailboxHandle[%d] | Address: 0x%016llx | Value: 0x%08lX",
            i, MmGetPhysicalAddress(MailboxHandle + i).QuadPart, mmio_read(MailboxHandle, i));
        mmio_write(MailboxHandle, i, 0x00000000);
    }
    mmio_write(MailboxHandle, MBOX_OFFSET_CONFIG, 0x00000400);
    mmio_write(MailboxHandle, MBOX_OFFSET_READ, 0x00000000);

    while ((mailbox_peek() & MBOX_FULL) != 0)
    {
        KeStallExecutionProcessor(MBOX_TIMEOUT);
        ++t1; if (t1 > MBOX_RETRIES) { return MBOX_FAILURE; }
    }

    mailbox_write(mail);

    while (1)
    {
        KeStallExecutionProcessor(MBOX_TIMEOUT);
        while ((mailbox_peek() & MBOX_EMPTY) != 0)
        {
            KeStallExecutionProcessor(MBOX_TIMEOUT);
            ++t2; if (t2 > MBOX_RETRIES) { return MBOX_FAILURE; }
        }

        checked = mailbox_read();
        if (mail == checked)
        {
            KeFlushIoBuffers(MailboxMiddle, TRUE, TRUE);
            return MBOX_SUCCESS;
        }
        ++t3; if (t3 > MBOX_RETRIES) { return MBOX_FAILURE; }
    }
}

unsigned int memory_allocate(unsigned int Size, unsigned int Align, unsigned int Flags)
{
    unsigned int i = 1;
    unsigned int a = 0;
    MailboxPacket[i++] = MBOX_REQUEST;

    MailboxPacket[i++] = MBOX_TAG_ALLOCATE_MEMORY;
    MailboxPacket[i++] = 12;      // Buffer Length
    MailboxPacket[i++] = 12;      // Data Length
a=i;MailboxPacket[i++] = Size;    // Value
    MailboxPacket[i++] = Align;   // Value
    MailboxPacket[i++] = Flags;   // Value

    MailboxPacket[i++] = 0;       // End Mark
    MailboxPacket[0] = i * 4;     // Update Packet Size

    if (MBOX_SUCCESS == mailbox_setup(8)) { return MailboxPacket[a]; }
    else { debug("[WARN]: Mailbox Transaction Error: t1 = %d, t2 = %d, t3 = %d", t1, t2, t3); return 0; }
}

unsigned int memory_reserve(unsigned int Handle, unsigned int Lock)
{
    unsigned int i = 1;
    unsigned int a = 0;
    MailboxPacket[i++] = MBOX_REQUEST;

    MailboxPacket[i++] = (Lock) ? MBOX_TAG_LOCK_MEMORY : MBOX_TAG_UNLOCK_MEMORY;
    MailboxPacket[i++] = 4;       // Buffer Length
    MailboxPacket[i++] = 4;       // Data Length
a=i;MailboxPacket[i++] = Handle;  // Value

    MailboxPacket[i++] = 0;       // End Mark
    MailboxPacket[0] = i * 4;     // Update Packet Size

    if (MBOX_SUCCESS == mailbox_setup(8)) { return MailboxPacket[a]; }
    else { debug("[WARN]: Mailbox Transaction Error: t1 = %d, t2 = %d, t3 = %d", t1, t2, t3); return 0; }
}

unsigned int memory_free(unsigned int Handle)
{
    unsigned int i = 1;
    unsigned int a = 0;
    MailboxPacket[i++] = MBOX_REQUEST;

    MailboxPacket[i++] = MBOX_TAG_RELEASE_MEMORY;
    MailboxPacket[i++] = 4;       // Buffer Length
    MailboxPacket[i++] = 4;       // Data Length
a=i;MailboxPacket[i++] = Handle;  // Value

    MailboxPacket[i++] = 0;       // End Mark
    MailboxPacket[0] = i * 4;     // Update Packet Size

    if (MBOX_SUCCESS == mailbox_setup(8)) { return MailboxPacket[a]; }
    else { debug("[WARN]: Mailbox Transaction Error: t1 = %d, t2 = %d, t3 = %d", t1, t2, t3); return 0; }
}

#pragma endregion
//////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region // PREPARATION

int gpu_fft_prepare(GPU_FFT& Handle, int Log2_N, int Direction, int Jobs)
{
    debug("[CALL]: gpu_fft_prepare");
    unsigned info_bytes, twid_bytes, data_bytes, code_bytes, unif_bytes, mail_bytes;
    unsigned size, *uptr, vc_tw, vc_data;
    int shared, unique, passes, error;

    if (gpu_fft_twiddle_size(Log2_N, &shared, &unique, &passes)) { return -2; }

    info_bytes = 4096;
    data_bytes = (1 + ((sizeof(GPU_FFT_COMPLEX) << Log2_N) | 4095));
    code_bytes = gpu_fft_shader_size(Log2_N);
    twid_bytes = sizeof(GPU_FFT_COMPLEX) * 16 * (shared + (GPU_FFT_QPUS * unique));
    unif_bytes = sizeof(int) * GPU_FFT_QPUS * (5 + (Jobs * 2));
    mail_bytes = sizeof(int) * GPU_FFT_QPUS * 2;

    size = info_bytes            // Header
         + data_bytes * Jobs * 2 // Ping-Pong Data, Aligned
         + code_bytes            // Shader, Aligned
         + twid_bytes            // Twiddles
         + unif_bytes            // Uniforms
         + mail_bytes;           // Mailbox Message

    debug("[QPU]: Allocating...");
    error = gpu_fft_allocate(Handle, size);
    if (error) { return error; }
    debug("[QPU]: Allocated");

    // Header
    PGPU_FFT info = (PGPU_FFT)Handle.share.vptr;
    PGPU_FFT_BASE base = (PGPU_FFT_BASE)info;
    gpu_fft_increment(Handle, info_bytes);

    // For Transpose
    info->x = 1 << Log2_N;
    info->y = Jobs;

    // Ping-pong buffers leave results in or out of place
    //info->input = info->output = Handle.share.cptr; //BSOD
    debug("[INFO]: info\t\t\t = %p", info);
    debug("[INFO]: info->input\t\t\t = %p", info->input);
    debug("[INFO]: info->output\t\t\t = %p", info->output);
    debug("[INFO]: &Handle\t\t\t = %p", &Handle);
    debug("[INFO]: &Handle.share\t\t\t = %p", &Handle.share);
    debug("[INFO]: &Handle.share.cptr\t\t\t = %p", &Handle.share.cptr);
    debug("[INFO]: Handle.share.cptr\t\t\t = %p", Handle.share.cptr);
    info->input = (PGPU_FFT_COMPLEX)Handle.share.cptr;
    info->output = (PGPU_FFT_COMPLEX)Handle.share.cptr;
    info->step = data_bytes / sizeof(GPU_FFT_COMPLEX);
    if (passes & 1) { info->output += (info->step * Jobs); } // odd => out of place
    vc_data = gpu_fft_increment(Handle, data_bytes * Jobs * 2);

    // Shader code
    memcpy(Handle.share.vptr, gpu_fft_shader_code(Log2_N), code_bytes);
    base->vc_code = gpu_fft_increment(Handle, code_bytes);

    // Twiddles
    gpu_fft_twiddle_data(Log2_N, Direction, Handle.share.fptr);
    vc_tw = gpu_fft_increment(Handle, twid_bytes);
    uptr = Handle.share.uptr;

    // Uniforms
    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        *uptr++ = vc_tw;
        *uptr++ = vc_tw + sizeof(GPU_FFT_COMPLEX) * 16 * (shared + (q * unique));
        *uptr++ = q;
        for (int i = 0; i < Jobs; ++i)
        {
            *uptr++ = vc_data + data_bytes * i;
            *uptr++ = vc_data + data_bytes * i + data_bytes * Jobs;
        }
        *uptr++ = 0;
        *uptr++ = (q == 0); // For mailbox: IRQ Enable, Master Only

        base->vc_uniforms[q] = gpu_fft_increment(Handle, sizeof(int) * (5 + (Jobs * 2)));
    }

    base->vc_msg = 0; // Direct Mode
    // Handle = *info; // Destructive???
    return 0;
}

int gpu_fft_execute(GPU_FFT& Handle)
{
    debug("[CALL]: gpu_fft_execute");
    unsigned int identity = 0x00000000;

    debug("[QPU]: Stage 0: Sanity Check");
    identity = mmio_read(V3DHandle, V3D_OFFSET_IDENT0);
    debug("[INFO]: V3D_IDENT0 = 0x%08lX", identity);
    if (identity != V3D_4D3V) { return -1; }

    debug("[QPU]: Stage 1: Disable Interrupts");
    mmio_write(V3DHandle, V3D_OFFSET_DBCFG,   0);          // Disallow IRQ
    mmio_write(V3DHandle, V3D_OFFSET_DBQITE,  0);          // Disable IRQ
    mmio_write(V3DHandle, V3D_OFFSET_DBQITC,  0xFFFFFFFF); // Resets IRQ flags

    debug("[QPU]: Stage 2: Clear Caches");
    mmio_write(V3DHandle, V3D_OFFSET_L2CACTL, 0x4);        // Clear L2 cache
    mmio_write(V3DHandle, V3D_OFFSET_SLCACTL, 0xFFFFFFFF); // Clear other caches

    debug("[QPU]: Stage 3: Reset Error States");
    mmio_write(V3DHandle, V3D_OFFSET_SRQCS,   0b1000000011000000); // Reset Error bits and Counts

    debug("[QPU]: Stage 4: Launch Shaders");
    for (int q = 0; q < GPU_FFT_QPUS; ++q) // Launch Shader(s)
    {
        mmio_write(V3DHandle, V3D_OFFSET_SRQUA, Handle.base.vc_uniforms[q]);
        mmio_write(V3DHandle, V3D_OFFSET_SRQPC, Handle.base.vc_code);
    }

    debug("[QPU]: Stage 5: Check Scoreboard");
    for (int i = 0; i < 100; ++i) // Busy-Wait Polling
    {
        // Scoreboard Format: 0xXXYY - XX: Total Jobs, YY: Failed Jobs
        unsigned int scoreboard = mmio_read(V3DHandle, V3D_OFFSET_SRQCS);
        if (((scoreboard >> 8) & 0xFF) == (unsigned int)(GPU_FFT_QPUS)) { break; } // All Done?
        debug("[INFO]: V3D_SRQCS = 0x%08lX", scoreboard);
    }

    debug("[QPU]: Stage 6: Finish Up");
    return 0;
}

#pragma endregion
//////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region // ALLOCATION

int gpu_fft_allocate(GPU_FFT& Handle, unsigned int Size)
{
    debug("[CALL]: gpu_fft_allocate");
    PHYSICAL_ADDRESS base;
    PHYSICAL_ADDRESS lowest;
    PHYSICAL_ADDRESS highest;

    // Map PowerHandle
    base.QuadPart = V3D_POWER_BASE;
    PowerHandle = (PULONG)MmMapIoSpace(base, V3D_POWER_LENGTH, MmNonCached);
    if (!PowerHandle) { return -4; }

    // Map ASBHandle
    base.QuadPart = V3D_ASB_BASE;
    ASBHandle = (PULONG)MmMapIoSpace(base, V3D_ASB_LENGTH, MmNonCached);
    if (!ASBHandle) { return -4; }

    // Map V3DHandle
    base.QuadPart = V3D_BASE;
    V3DHandle = (PULONG)MmMapIoSpace(base, V3D_LENGTH, MmNonCached);
    if (!V3DHandle) { return -4; }

    // Map MailboxHandle
    base.QuadPart = MBOX_BASE;
    MailboxHandle = (PULONG)MmMapIoSpace(base, MBOX_LENGTH, MmNonCached);
    if (!MailboxHandle) { return -4; }

    // Power On the V3D
    v3d_setup(1);

    // Allocate Mailbox Packet
    base.QuadPart = MBOX_MAX;
    MailboxPacket = (PULONG)MmAllocateContiguousMemory(MBOX_MTU, base);
    if (!MailboxPacket) { return -5; }
    for (int i = 0; i < (MBOX_MTU / 4); ++i) { MailboxPacket[i] = 0; }

    // Allocate Mailbox Middle
    lowest.QuadPart = MBOX_MIN;
    highest.QuadPart = MBOX_MAX;
    base.QuadPart = 0x0; //SKIP
    MailboxMiddle = MmAllocatePagesForMdlEx(lowest, highest, base,
        MBOX_MTU, MmNonCached, MM_ALLOCATE_REQUIRE_CONTIGUOUS_CHUNKS);
    if (!MailboxMiddle) { return -5; }

    // Allocate Shared Memory
    Handle.base.shared_size = Size;
    Handle.base.shared_handle = memory_allocate(Handle.base.shared_size, GPU_FFT_ALIGN, GPU_FFT_NO_FLUSH);
    if (!Handle.base.shared_handle) { return -3; }

    // Lock Shared Memory For Use
    base.QuadPart = VC_TRANSLATE(Handle.share.vc);
    Handle.share.vc = memory_reserve(Handle.base.shared_handle, 1);
    Handle.share.vptr = (PULONG)MmMapIoSpace(base, Handle.base.shared_size, MmNonCached);
    if (!Handle.share.vptr) { memory_free(Handle.base.shared_handle); return -4; }

    return 0;
}

int gpu_fft_increment(GPU_FFT& Handle, int Bytes)
{
    debug("[CALL]: gpu_fft_increment");
    int vc = Handle.share.vc;
    Handle.share.vc += Bytes;
    Handle.share.bptr += Bytes;
    return vc;
}

void gpu_fft_release(GPU_FFT& Handle)
{
    debug("[CALL]: gpu_fft_release");
    memory_reserve(Handle.base.shared_handle, 0);
    memory_free(Handle.base.shared_handle);
    MmUnmapIoSpace(Handle.share.vptr, Handle.base.shared_size);
    MmFreePagesFromMdl(MailboxMiddle);
    MmFreeContiguousMemory(MailboxPacket);
    v3d_setup(0); // Power Off V3D
    MmUnmapIoSpace(MailboxHandle, MBOX_LENGTH);
    MmUnmapIoSpace(V3DHandle, V3D_LENGTH);
    MmUnmapIoSpace(ASBHandle, V3D_ASB_LENGTH);
    MmUnmapIoSpace(PowerHandle, V3D_POWER_LENGTH);
}

#pragma endregion
//////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region // SHADERS

static unsigned int shader_256[] =
{
    #include "hex/shader_256.hex"
};

static unsigned int shader_512[] =
{
    #include "hex/shader_512.hex"
};

static unsigned int shader_1k[] =
{
    #include "hex/shader_1k.hex"
};

static unsigned int shader_2k[] =
{
    #include "hex/shader_2k.hex"
};

static unsigned int shader_4k[] =
{
    #include "hex/shader_4k.hex"
};

static unsigned int shader_8k[] =
{
    #include "hex/shader_8k.hex"
};

static unsigned int shader_16k[] =
{
    #include "hex/shader_16k.hex"
};

static unsigned int shader_32k[] =
{
    #include "hex/shader_32k.hex"
};

static unsigned int shader_64k[] =
{
    #include "hex/shader_64k.hex"
};

static unsigned int shader_128k[] =
{
    #include "hex/shader_128k.hex"
};

static unsigned int shader_256k[] =
{
    #include "hex/shader_256k.hex"
};

static unsigned int shader_512k[] =
{
    #include "hex/shader_512k.hex"
};

static unsigned int shader_1024k[] =
{
    #include "hex/shader_1024k.hex"
};

static unsigned int shader_2048k[] =
{
    #include "hex/shader_2048k.hex"
};

static unsigned int shader_4096k[] =
{
    #include "hex/shader_4096k.hex"
};

static SHADERS shaders[] =
{
    { sizeof(shader_256),   shader_256   },
    { sizeof(shader_512),   shader_512   },
    { sizeof(shader_1k),    shader_1k    },
    { sizeof(shader_2k),    shader_2k    },
    { sizeof(shader_4k),    shader_4k    },
    { sizeof(shader_8k),    shader_8k    },
    { sizeof(shader_16k),   shader_16k   },
    { sizeof(shader_32k),   shader_32k   },
    { sizeof(shader_64k),   shader_64k   },
    { sizeof(shader_128k),  shader_128k  },
    { sizeof(shader_256k),  shader_256k  },
    { sizeof(shader_512k),  shader_512k  },
    { sizeof(shader_1024k), shader_1024k },
    { sizeof(shader_2048k), shader_2048k },
    { sizeof(shader_4096k), shader_4096k }
};

unsigned int  gpu_fft_shader_size(int log2_N) { return shaders[log2_N - 8].size; }
unsigned int* gpu_fft_shader_code(int log2_N) { return shaders[log2_N - 8].code; }

#pragma endregion
//////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region // TWIDDLES

static float* twiddles_base_16(double two_pi, float* out, double theta)
{
    for (int i = 0; i < 16; ++i)
    {
        *out++ = (float)cos(two_pi / 16 * k[i] * m[i] + theta * k[i]);
        *out++ = (float)sin(two_pi / 16 * k[i] * m[i] + theta * k[i]);
    }
    return out;
}

static float* twiddles_base_32(double two_pi, float* out, double theta)
{
    for (int i = 0; i < 16; ++i)
    {
        *out++ = (float)cos(two_pi / 32 * i + theta);
        *out++ = (float)sin(two_pi / 32 * i + theta);
    }
    return twiddles_base_16(two_pi, out, 2 * theta);
}

static float* twiddles_base_64(double two_pi, float* out)
{
    for (int i = 0; i < 32; ++i)
    {
        *out++ = (float)cos(two_pi / 64 * i);
        *out++ = (float)sin(two_pi / 64 * i);
    }
    return twiddles_base_32(two_pi, out, 0);
}

static float* twiddles_step_16(double two_pi, float* out, double theta)
{
    for (int i = 0; i < 16; ++i)
    {
        *out++ = (float)ALPHA(theta * k[i]);
        *out++ = (float)BETA(theta * k[i]);
    }
    two_pi = two_pi;
    return out;
}

static float* twiddles_step_32(double two_pi, float* out, double theta)
{
    for (int i = 0; i < 16; ++i)
    {
        *out++ = (float)ALPHA(theta);
        *out++ = (float)BETA(theta);
    }
    return twiddles_step_16(two_pi, out, 2 * theta);
}

static float* twiddles_step_64(double two_pi, float* out, double theta)
{
    for (int i = 0; i < 32; ++i)
    {
        *out++ = (float)ALPHA(theta);
        *out++ = (float)BETA(theta);
    }
    return twiddles_step_32(two_pi, out, 2 * theta);
}

static void twiddles_256(double two_pi, float* out)
{
    double N = 256;
    out = twiddles_base_16(two_pi, out, 0);
    out = twiddles_step_16(two_pi, out, two_pi / N * GPU_FFT_QPUS);

    for (int q = 0; q < GPU_FFT_QPUS; q++)
    {
        out = twiddles_base_16(two_pi, out, two_pi / N * q);
    }
}

static void twiddles_512(double two_pi, float* out)
{
    double N = 512;
    out = twiddles_base_32(two_pi, out, 0);
    out = twiddles_step_16(two_pi, out, two_pi / N * GPU_FFT_QPUS);

    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        out = twiddles_base_16(two_pi, out, two_pi / N * q);
    }
}

static void twiddles_1k(double two_pi, float* out)
{
    double N = 1024;
    out = twiddles_base_32(two_pi, out, 0);
    out = twiddles_step_32(two_pi, out, two_pi / N * GPU_FFT_QPUS);

    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        out = twiddles_base_32(two_pi, out, two_pi / N * q);
    }
}

static void twiddles_2k(double two_pi, float* out)
{
    double N = 2048;
    out = twiddles_base_64(two_pi, out);
    out = twiddles_step_32(two_pi, out, two_pi / N * GPU_FFT_QPUS);

    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        out = twiddles_base_32(two_pi, out, two_pi / N * q);
    }
}

static void twiddles_4k(double two_pi, float* out)
{
    double N = 4096;
    out = twiddles_base_16(two_pi, out, 0);
    out = twiddles_step_16(two_pi, out, two_pi / N * 16);
    out = twiddles_step_16(two_pi, out, two_pi / N * GPU_FFT_QPUS);

    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        out = twiddles_base_16(two_pi, out, two_pi / N * q);
    }
}

static void twiddles_8k(double two_pi, float* out)
{
    double N = 8192;
    out = twiddles_base_32(two_pi, out, 0);
    out = twiddles_step_16(two_pi, out, two_pi / N * 16);
    out = twiddles_step_16(two_pi, out, two_pi / N * GPU_FFT_QPUS);

    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        out = twiddles_base_16(two_pi, out, two_pi / N * q);
    }
}

static void twiddles_16k(double two_pi, float* out)
{
    double N = 16384;
    out = twiddles_base_32(two_pi, out, 0);
    out = twiddles_step_32(two_pi, out, two_pi / N * 16);
    out = twiddles_step_16(two_pi, out, two_pi / N * GPU_FFT_QPUS);

    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        out = twiddles_base_16(two_pi, out, two_pi / N * q);
    }
}

static void twiddles_32k(double two_pi, float* out)
{
    double N = 32768;
    out = twiddles_base_32(two_pi, out, 0);
    out = twiddles_step_32(two_pi, out, two_pi / N * 32);
    out = twiddles_step_32(two_pi, out, two_pi / N * GPU_FFT_QPUS);

    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        out = twiddles_base_32(two_pi, out, two_pi / N * q);
    }
}

static void twiddles_64k(double two_pi, float* out)
{
    double N = 65536;
    out = twiddles_base_64(two_pi, out);
    out = twiddles_step_32(two_pi, out, two_pi / N * 32);
    out = twiddles_step_32(two_pi, out, two_pi / N * GPU_FFT_QPUS);
    
    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        out = twiddles_base_32(two_pi, out, two_pi / N * q);
    }
}

static void twiddles_128k(double two_pi, float* out)
{
    double N = 128 * 1024;
    out = twiddles_base_32(two_pi, out, 0);
    out = twiddles_step_16(two_pi, out, two_pi / N * 16 * 16);
    out = twiddles_step_16(two_pi, out, two_pi / N * 16);
    out = twiddles_step_16(two_pi, out, two_pi / N * GPU_FFT_QPUS);

    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        out = twiddles_base_16(two_pi, out, two_pi / N * q);
    }
}

static void twiddles_256k(double two_pi, float* out)
{
    double N = 256 * 1024;
    out = twiddles_base_32(two_pi, out, 0);
    out = twiddles_step_16(two_pi, out, two_pi / N * 32 * 16);
    out = twiddles_step_16(two_pi, out, two_pi / N * 32);
    out = twiddles_step_32(two_pi, out, two_pi / N * GPU_FFT_QPUS);

    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        out = twiddles_base_32(two_pi, out, two_pi / N * q);
    }
}

static void twiddles_512k(double two_pi, float* out)
{
    double N = 512 * 1024;
    out = twiddles_base_32(two_pi, out, 0);
    out = twiddles_step_16(two_pi, out, two_pi / N * 32 * 32);
    out = twiddles_step_32(two_pi, out, two_pi / N * 32);
    out = twiddles_step_32(two_pi, out, two_pi / N * GPU_FFT_QPUS);

    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        out = twiddles_base_32(two_pi, out, two_pi / N * q);
    }
}

static void twiddles_1024k(double two_pi, float* out)
{
    double N = 1024 * 1024;
    out = twiddles_base_32(two_pi, out, 0);
    out = twiddles_step_32(two_pi, out, two_pi / N * 32 * 32);
    out = twiddles_step_32(two_pi, out, two_pi / N * 32);
    out = twiddles_step_32(two_pi, out, two_pi / N * GPU_FFT_QPUS);

    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        out = twiddles_base_32(two_pi, out, two_pi / N * q);
    }
}

static void twiddles_2048k(double two_pi, float* out)
{
    double N = 2048 * 1024;
    out = twiddles_base_64(two_pi, out);
    out = twiddles_step_32(two_pi, out, two_pi / N * 32 * 32);
    out = twiddles_step_32(two_pi, out, two_pi / N * 32);
    out = twiddles_step_32(two_pi, out, two_pi / N * GPU_FFT_QPUS);

    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        out = twiddles_base_32(two_pi, out, two_pi / N * q);
    }
}

static void twiddles_4096k(double two_pi, float* out)
{
    double N = 4096 * 1024;
    out = twiddles_base_64(two_pi, out);
    out = twiddles_step_64(two_pi, out, two_pi / N * 32 * 32);
    out = twiddles_step_32(two_pi, out, two_pi / N * 32);
    out = twiddles_step_32(two_pi, out, two_pi / N * GPU_FFT_QPUS);

    for (int q = 0; q < GPU_FFT_QPUS; ++q)
    {
        out = twiddles_base_32(two_pi, out, two_pi / N * q);
    }
}

static TWIDDLES twiddles[] =
{
    { 2, 2, 1, twiddles_256   },
    { 2, 3, 1, twiddles_512   },
    { 2, 4, 2, twiddles_1k    },
    { 2, 6, 2, twiddles_2k    },
    { 3, 3, 1, twiddles_4k    },
    { 3, 4, 1, twiddles_8k    },
    { 3, 5, 1, twiddles_16k   },
    { 3, 6, 2, twiddles_32k   },
    { 3, 8, 2, twiddles_64k   },
    { 4, 5, 1, twiddles_128k  },
    { 4, 6, 2, twiddles_256k  },
    { 4, 7, 2, twiddles_512k  },
    { 4, 8, 2, twiddles_1024k },
    { 4,10, 2, twiddles_2048k },
    { 4,12, 2, twiddles_4096k }
};

int gpu_fft_twiddle_size(int log2_N, int* shared, int* unique, int* passes)
{
    if ((log2_N < 8) || (log2_N > 22)) { return -1; }
    *shared = twiddles[log2_N - 8].shared;
    *unique = twiddles[log2_N - 8].unique;
    *passes = twiddles[log2_N - 8].passes;
    return 0;
}

void gpu_fft_twiddle_data(int log2_N, int direction, float* out)
{
    twiddles[log2_N - 8].twiddles((direction == GPU_FFT_FWD ? -2 : 2) * GPU_FFT_PI, out);
}

#pragma endregion
    //////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                          //
  //                                  (V) WoR-Project 2021                                    //
 //                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////