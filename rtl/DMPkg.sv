`ifndef DM_PKG
`define DM_PKG

package DMPkg;

    typedef enum logic {
        WRITE_THROUGH = 1'b0,
        WRITE_BACK    = 1'b1
    } write_mode_t;

    typedef enum logic [3:0] {
        IDLE  = 4'b0001,
        FILL  = 4'b0010,    /* A AcquireBlock, D GrantData beats into the cache, E GrantAck */
        EVICT = 4'b0100,    /* C ReleaseData beats from the cache, D ReleaseAck             */
        PUT   = 4'b1000     /* D AccessAck of a write-through PutPartialData                */
    } fsm_t;

    typedef enum logic [2:0] {
        P_IDLE = 3'b001,
        P_ACK  = 3'b010,    /* C ProbeAck                     */
        P_DATA = 3'b100     /* C ProbeAckData beats from the cache */
    } probe_fsm_t;

    /* Permissions of a cached line, in increasing order (spec 9.1) */
    typedef enum logic [1:0] {
        PERM_N = 2'd0,
        PERM_B = 2'd1,
        PERM_T = 2'd2
    } perm_t;

    /* TileLink 1.9.3 opcodes (Table 13) */
    localparam logic [2:0] TL_A_PUT_PARTIAL_DATA = 3'd1;
    localparam logic [2:0] TL_A_ACQUIRE_BLOCK    = 3'd6;
    localparam logic [2:0] TL_B_PROBE_BLOCK      = 3'd6;
    localparam logic [2:0] TL_C_PROBE_ACK        = 3'd4;
    localparam logic [2:0] TL_C_PROBE_ACK_DATA   = 3'd5;
    localparam logic [2:0] TL_C_RELEASE_DATA     = 3'd7;
    localparam logic [2:0] TL_D_ACCESS_ACK       = 3'd0;
    localparam logic [2:0] TL_D_GRANT_DATA       = 3'd5;
    localparam logic [2:0] TL_D_RELEASE_ACK      = 3'd6;

    /* TileLink 1.9.3 permission transfers (Table 31) */
    localparam logic [2:0] TL_CAP_TOT    = 3'd0;
    localparam logic [2:0] TL_CAP_TOB    = 3'd1;
    localparam logic [2:0] TL_CAP_TON    = 3'd2;
    localparam logic [2:0] TL_GROW_NTOB  = 3'd0;
    localparam logic [2:0] TL_GROW_NTOT  = 3'd1;
    localparam logic [2:0] TL_PRUNE_TTOB = 3'd0;
    localparam logic [2:0] TL_PRUNE_TTON = 3'd1;
    localparam logic [2:0] TL_PRUNE_BTON = 3'd2;
    localparam logic [2:0] TL_REPORT_TTOT = 3'd3;
    localparam logic [2:0] TL_REPORT_BTOB = 3'd4;
    localparam logic [2:0] TL_REPORT_NTON = 3'd5;

endpackage

`endif
