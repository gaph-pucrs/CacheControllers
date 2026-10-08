`ifndef DM_PKG
`define DM_PKG

package DMPkg;

    typedef enum logic {
        WRITE_THROUGH = 1'b0,
        WRITE_BACK    = 1'b1
    } write_mode_t;

    typedef enum logic [2:0] {
        IDLE  = 3'b001,
        FILL  = 3'b010,
        EVICT = 3'b100
    } fsm_t;

endpackage

`endif
