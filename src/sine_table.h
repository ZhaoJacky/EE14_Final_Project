// Table that contains the digital data for our sine wave.
int sine[360] = {

    0x800,0x823,0x847,0x86b,0x88e,
    0x8b2,0x8d6,0x8f9,0x91d,0x940,
    0x963,0x986,0x9a9,0x9cc,0x9ef,
    0xa12,0xa34,0xa56,0xa78,0xa9a,
    0xabc,0xadd,0xaff,0xb20,0xb40,
    0xb61,0xb81,0xba1,0xbc1,0xbe0,
    0xc00,0xc1e,0xc3d,0xc5b,0xc79,
    0xc96,0xcb3,0xcd0,0xcec,0xd08,
    0xd24,0xd3f,0xd5a,0xd74,0xd8e,
    0xda8,0xdc1,0xdd9,0xdf1,0xe09,
    0xe20,0xe37,0xe4d,0xe63,0xe78,
    0xe8d,0xea1,0xeb5,0xec8,0xedb,
    0xeed,0xeff,0xf10,0xf20,0xf30,
    0xf40,0xf4e,0xf5d,0xf6a,0xf77,
    0xf84,0xf90,0xf9b,0xfa6,0xfb0,
    0xfba,0xfc3,0xfcb,0xfd3,0xfda,
    0xfe0,0xfe6,0xfec,0xff0,0xff4,
    0xff8,0xffb,0xffd,0xffe,0xfff,
    0xfff,0xfff,0xffe,0xffd,0xffb,
    0xff8,0xff4,0xff0,0xfec,0xfe6,
    0xfe0,0xfda,0xfd3,0xfcb,0xfc3,
    0xfba,0xfb0,0xfa6,0xf9b,0xf90,
    0xf84,0xf77,0xf6a,0xf5d,0xf4e,
    0xf40,0xf30,0xf20,0xf10,0xeff,
    0xeed,0xedb,0xec8,0xeb5,0xea1,
    0xe8d,0xe78,0xe63,0xe4d,0xe37,
    0xe20,0xe09,0xdf1,0xdd9,0xdc1,
    0xda8,0xd8e,0xd74,0xd5a,0xd3f,
    0xd24,0xd08,0xcec,0xcd0,0xcb3,
    0xc96,0xc79,0xc5b,0xc3d,0xc1e,
    0xc00,0xbe0,0xbc1,0xba1,0xb81,
    0xb61,0xb40,0xb20,0xaff,0xadd,
    0xabc,0xa9a,0xa78,0xa56,0xa34,
    0xa12,0x9ef,0x9cc,0x9a9,0x986,
    0x963,0x940,0x91d,0x8f9,0x8d6,
    0x8b2,0x88e,0x86b,0x847,0x823,
    0x800,0x7dc,0x7b8,0x794,0x771,
    0x74d,0x729,0x706,0x6e2,0x6bf,
    0x69c,0x679,0x656,0x633,0x610,
    0x5ed,0x5cb,0x5a9,0x587,0x565,
    0x543,0x522,0x500,0x4df,0x4bf,
    0x49e,0x47e,0x45e,0x43e,0x41f,
    0x400,0x3e1,0x3c2,0x3a4,0x386,
    0x369,0x34c,0x32f,0x313,0x2f7,
    0x2db,0x2c0,0x2a5,0x28b,0x271,
    0x257,0x23e,0x226,0x20e,0x1f6,
    0x1df,0x1c8,0x1b2,0x19c,0x187,
    0x172,0x15e,0x14a,0x137,0x124,
    0x112,0x100,0x0ef,0x0df,0x0cf,
    0x0bf,0x0b1,0x0a2,0x095,0x088,
    0x07b,0x06f,0x064,0x059,0x04f,
    0x045,0x03c,0x034,0x02c,0x025,
    0x01f,0x019,0x013,0x00f,0x00b,
    0x007,0x004,0x002,0x001,0x000,
    0x000,0x000,0x001,0x002,0x004,
    0x007,0x00b,0x00f,0x013,0x019,
    0x01f,0x025,0x02c,0x034,0x03c,
    0x045,0x04f,0x059,0x064,0x06f,
    0x07b,0x088,0x095,0x0a2,0x0b1,
    0x0bf,0x0cf,0x0df,0x0ef,0x100,
    0x112,0x124,0x137,0x14a,0x15e,
    0x172,0x187,0x19c,0x1b2,0x1c8,
    0x1df,0x1f6,0x20e,0x226,0x23e,
    0x257,0x271,0x28b,0x2a5,0x2c0,
    0x2db,0x2f7,0x313,0x32f,0x34c,
    0x369,0x386,0x3a4,0x3c2,0x3e1,
    0x400,0x41f,0x43e,0x45e,0x47e,
    0x49e,0x4bf,0x4df,0x500,0x522,
    0x543,0x565,0x587,0x5a9,0x5cb,
    0x5ed,0x610,0x633,0x656,0x679,
    0x69c,0x6bf,0x6e2,0x706,0x729,
    0x74d,0x771,0x794,0x7b8,0x7dc
};

int sine2[180] = {0xd24,0xd5a,0xd8e,0xdc1,0xdf1,
    0xe20,0xe4d,0xe78,0xea1,0xec8,
    0xeed,0xf10,0xf30,0xf4e,0xf6a,
    0xf84,0xf9b,0xfb0,0xfc3,0xfd3,
    0xfe0,0xfec,0xff4,0xffb,0xffe,
    0xfff,0xffe,0xffb,0xff4,0xfec,
    0xfe0,0xfd3,0xfc3,0xfb0,0xf9b,
    0xf84,0xf6a,0xf4e,0xf30,0xf10,
    0xeed,0xec8,0xea1,0xe78,0xe4d,
    0xe20,0xdf1,0xdc1,0xd8e,0xd5a,
    0xd24,0xcec,0xcb3,0xc79,0xc3d,
    0xc00,0xbc1,0xb81,0xb40,0xaff,
    0xabc,0xa78,0xa34,0x9ef,0x9a9,
    0x963,0x91d,0x8d6,0x88e,0x847,
    0x800,0x7b8,0x771,0x729,0x6e2,
    0x69c,0x656,0x610,0x5cb,0x587,
    0x543,0x500,0x4bf,0x47e,0x43e,
    0x400,0x3c2,0x386,0x34c,0x313,
    0x2db,0x2a5,0x271,0x23e,0x20e,
    0x1df,0x1b2,0x187,0x15e,0x137,
    0x112,0x0ef,0x0cf,0x0b1,0x095,
    0x07b,0x064,0x04f,0x03c,0x02c,
    0x01f,0x013,0x00b,0x004,0x001,
    0x000,0x001,0x004,0x00b,0x013,
    0x01f,0x02c,0x03c,0x04f,0x064,
    0x07b,0x095,0x0b1,0x0cf,0x0ef,
    0x112,0x137,0x15e,0x187,0x1b2,
    0x1df,0x20e,0x23e,0x271,0x2a5,
    0x2db,0x313,0x34c,0x386,0x3c2,
    0x400,0x43e,0x47e,0x4bf,0x500,
    0x543,0x587,0x5cb,0x610,0x656,
    0x69c,0x6e2,0x729,0x771,0x7b8};