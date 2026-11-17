//------------------------------------------------------------------------------
// IVAN PALADIN        - 07/OUT/2026
// ANGELO DAL ZOTTO    - 08/OUT/2026
//------------------------------------------------------------------------------
// Simulation-only monitor for DMCtrl. Reads DMCtrl internals through upward
// references (DMCtrl.<signal>), so it only works bound into DMCtrl, with the
// same parameters as the bound instance:
//     bind DMCtrl DMDebug #(.ADDR_WIDTH(ADDR_WIDTH), .CACHE_WIDTH(CACHE_WIDTH),
//                           .OFFSET_WIDTH(OFFSET_WIDTH), .WMODE(WMODE)) u_debug ();
// Logs every FILL, EVICT, DIRTY and PROBE event as CSV to ./debug/<instance path>.csv:
//     time,event,line,tag
// A PROBE is logged when accepted, with the permission it leaves: PROBE_N, _B or _T
// * GAPH - Hardware Design Support Group
// * PUCRS - Pontifical Catholic University of Rio Grande do Sul <https://pucrs.br/>
//------------------------------------------------------------------------------

`include "DMPkg.sv"

module DMDebug
    import DMPkg::*;
#(
    /* Must be the bound DMCtrl's parameters (passed by the bind). They can't be read */
    /* as DMCtrl.<param>: constants can't be hierarchical (IEEE 1800 6.20.2), and     */
    /* each cache configuration needs its own DMDebug copy, or the widths read        */
    /* through DMCtrl.<signal> are fixed once for all caches. The widths are unused   */
    /* here, but they set the widths of DMCtrl.tag/line_idx, so they must still       */
    /* split the copies.                                                              */
    /* verilator lint_off UNUSEDPARAM */
    parameter int unsigned ADDR_WIDTH   = 20,
    parameter int unsigned CACHE_WIDTH  = 12,
    parameter int unsigned OFFSET_WIDTH = 6,
    parameter write_mode_t WMODE        = WRITE_BACK
    /* verilator lint_on UNUSEDPARAM */
);

    logic fill_done, evict_done, dirty_set, probe_done;

    assign fill_done  = DMCtrl.end_fill;
    assign evict_done = DMCtrl.end_evict;
    assign dirty_set  = DMCtrl.dirty_set;
    assign probe_done = DMCtrl.probe_accept && DMCtrl.probe_hit;

    string file_name;
    int    fd;

    /* One file per instance: icache and dcache must not share (and clobber) the same file */
    initial begin
        file_name = $sformatf("./debug/%m.csv");
        fd        = $fopen(file_name, "w");
        if (fd == 0)
            $display("[DMDebug] Could not open %s", file_name);
        else
            $fdisplay(fd, "time,event,line,tag");
    end

    always @(posedge DMCtrl.clk) begin
        if (fd != 0) begin
            if (fill_done)
                $fdisplay(fd, "%0d,FILL,%0d,%0d",  $time, DMCtrl.line_idx, DMCtrl.tag);
            if (evict_done)
                $fdisplay(fd, "%0d,EVICT,%0d,%0d", $time, DMCtrl.line_idx, DMCtrl.evict_tag);
            if (dirty_set)
                $fdisplay(fd, "%0d,DIRTY,%0d,%0d", $time, DMCtrl.line_idx, DMCtrl.tag);
            if (probe_done)
                $fdisplay(fd, "%0d,PROBE_%s,%0d,%0d", $time, DMCtrl.probe_new.name().substr(5, 5),
                          DMCtrl.b_index, DMCtrl.b_tag);
        end
    end

    final begin
        if (fd != 0)
            $fclose(fd);
    end

endmodule
