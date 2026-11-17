`ifndef DM_PKG
`define DM_PKG

package DMPkg;

    typedef enum logic {
        WRITE_THROUGH = 1'b0,
        WRITE_BACK    = 1'b1
    } write_mode_t;

    typedef enum logic [4:0] {
        IDLE      = 5'b00001,
        FILL      = 5'b00010,   /* A AcquireBlock, D GrantData beats into the cache */
        GRANT_ACK = 5'b00100,   /* E GrantAck                                       */
        EVICT     = 5'b01000,   /* C ReleaseData beats from the cache, D ReleaseAck */
        PUT       = 5'b10000    /* D AccessAck of a write-through PutPartialData    */
    } fsm_t;

    /* TileLink 1.9.3 opcodes (Table 13) */
    localparam logic [2:0] TL_A_PUT_PARTIAL_DATA = 3'd1;
    localparam logic [2:0] TL_A_ACQUIRE_BLOCK    = 3'd6;
    localparam logic [2:0] TL_C_RELEASE_DATA     = 3'd7;
    localparam logic [2:0] TL_D_ACCESS_ACK       = 3'd0;
    localparam logic [2:0] TL_D_GRANT_DATA       = 3'd5;
    localparam logic [2:0] TL_D_RELEASE_ACK      = 3'd6;

    /* TileLink 1.9.3 permission transfers (Table 31) */
    localparam logic [2:0] TL_GROW_NTOT  = 3'd1;
    localparam logic [2:0] TL_PRUNE_TTON = 3'd1;

endpackage

`endif
