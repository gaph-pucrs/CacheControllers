`ifndef DM_PKG
`define DM_PKG

package DMPkg;

    typedef enum logic {
        WRITE_THROUGH = 1'b0,
        WRITE_BACK    = 1'b1
    } write_mode_t;

endpackage

`endif
