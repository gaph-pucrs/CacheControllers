# CacheControllers

A small collection of **direct-mapped cache controllers** for the RS5 processor
(and any core that speaks the same simple synchronous-memory protocol). Each
controller is a thin, synthesizable RTL block that turns a single core memory
port into a cached port, using two external memories:

* a small, fast **cache SRAM** that holds the line data;
* a large, slower **main memory** that holds everything.

All metadata (valid / dirty / tag) lives in flip-flops inside the controller; the
SRAMs store only data. The controllers are parameterizable and contain no
vendor-specific primitives.

| Module        | Policy                          | Allocates on write | Dirty bits | Data-bus widths      |
|---------------|---------------------------------|--------------------|------------|----------------------|
| `DMWTCtrl`    | Direct-mapped **write-through** | no (write-no-allocate) | no     | fixed 32-bit         |
| `DMWBCtrl`    | Direct-mapped **write-back**    | yes (write-allocate)   | yes    | parameterizable      |

---

## 1. The memory model

Both controllers present three interfaces. The data widths shown are the default
(scalar) configuration.

```
                 ┌──────────────────────────┐
   core  ───────▶│  ce_i / we_i / address_i │
   (CPU /        │  data_i  ──▶              │            ┌───────────────┐
    vector) ◀────│  data_o  / busy_o        │  cache_*   │   cache SRAM   │
                 │                          │◀──────────▶│  (line data)  │
                 │       controller          │            └───────────────┘
                 │   (tags + valid/dirty,    │  mem_*     ┌───────────────┐
                 │    miss/burst control)    │◀──────────▶│  main memory  │
                 └──────────────────────────┘            └───────────────┘
```

### Core interface

| Signal       | Dir | Meaning                                                        |
|--------------|-----|----------------------------------------------------------------|
| `ce_i`       | in  | Access enable (the core wants a load or a store this cycle)    |
| `we_i`       | in  | Per-byte write enable; all-zero means a read                   |
| `address_i`  | in  | Byte address (`ADDR_WIDTH` bits)                               |
| `data_i`     | in  | Write data                                                     |
| `data_o`     | out | Read data (valid the cycle after the access, like a sync RAM)  |
| `busy_o`     | out | Stall: the access has not completed; hold the request stable   |

The core behaves like it is talking to a 1-cycle-latency memory. On a **hit** the
controller never asserts `busy_o`; the access proceeds with the normal one-cycle
read latency. On a **miss** the controller asserts `busy_o` while it services the
miss, and the core must hold `ce_i`/`we_i`/`address_i`/`data_i` stable until
`busy_o` drops.

### Cache SRAM interface (`cache_*`)

A synchronous, byte-writable SRAM used as the line data store. The controller
assumes a **fixed 1-cycle read latency** and that the SRAM never stalls. (Adding
a `cache_busy_i` for a slower SRAM would require output buffering with bypass —
see the `@todo` in the sources.) `cache_addr_o` is a **byte address** into the
store; its size is `CACHE_WIDTH` bits, i.e. the store holds `2^CACHE_WIDTH` bytes.

### Main memory interface (`mem_*`)

A synchronous, byte-writable memory with a variable-latency handshake:
`mem_busy_i` is high while the access has not completed. The controller keeps the
request asserted and only advances when it samples `mem_busy_i` low. This matches
the RS5 testbench memory model (`sim/RAM_mem.sv` + the delay process in the
testbenches).

---

## 2. Address decode (common to both controllers)

An address is split into tag / index / offset:

```
            ADDR_WIDTH
 ┌───────────────┬───────────────┬───────────────┐
 │      tag      │     index     │    offset     │
 └───────────────┴───────────────┴───────────────┘
   TAG_WIDTH        INDEX_WIDTH     OFFSET_WIDTH
```

Derived from the three structural parameters:

| Parameter      | Meaning                                             |
|----------------|-----------------------------------------------------|
| `ADDR_WIDTH`   | Main-memory address width (bits)                    |
| `CACHE_WIDTH`  | log2 of the cache data store size in **bytes**      |
| `OFFSET_WIDTH` | log2 of the **line size** in bytes                  |

```
TAG_WIDTH   = ADDR_WIDTH  - CACHE_WIDTH      // bits compared to detect a hit
INDEX_WIDTH = CACHE_WIDTH - OFFSET_WIDTH     // selects the line (NB_BLOCKS = 2^INDEX_WIDTH)
LINE_BYTES  = 2^OFFSET_WIDTH                 // bytes per line
```

Example (RS5 default): `ADDR_WIDTH=28`, `CACHE_WIDTH=12`, `OFFSET_WIDTH=6`
→ 16-bit tags, 64 lines of 64 bytes = a 4 KiB direct-mapped cache.

Being **direct-mapped**, the victim of any miss is always the single line stored
at that index; there is no replacement policy.

---

## 3. `DMWTCtrl` — write-through controller

Policy: **write-through, write-no-allocate.**

| Event       | Behaviour                                                            |
|-------------|----------------------------------------------------------------------|
| Read hit    | Return the word from the cache SRAM.                                  |
| Read miss   | Fill the line from main memory, then return the word.                |
| Write hit   | Write the word to **both** the cache SRAM and main memory.           |
| Write miss  | Write the word **straight to main memory**; the line is not allocated.|

Because every store reaches main memory immediately, the cache copy and memory
are always consistent — no dirty state is needed, and a line can be replaced with
no write-back. The cost is memory write traffic on every store.

Miss handling uses a single `filling` flag with two word pointers (`fetch_idx`
issuing reads, `fill_idx` writing the returned words into the SRAM), streaming the
line in at roughly one word per cycle. `busy_o` is asserted during a read miss /
fill and while a write-through is waiting on `mem_busy_i`.

---

## 4. `DMWBCtrl` — write-back controller

Policy: **write-back, write-allocate.** Each line additionally carries a **dirty**
bit.

| Event       | Behaviour                                                                       |
|-------------|---------------------------------------------------------------------------------|
| Read hit    | Return the word from the cache SRAM.                                            |
| Read miss   | **Evict** the victim if it is dirty, **fill** the new line, then return the word.|
| Write hit   | Write the word to the cache SRAM and set the line **dirty** (no memory access). |
| Write miss  | Evict if dirty, fill the line, then write the word and set it dirty.            |

Main memory is written **only** when a dirty line is evicted, which sharply
reduces write traffic versus write-through (on the RS5 riscv-tests suite: 32
memory writes vs 1172 for write-through). Write-allocate is required for
correctness with sub-word stores (`sb`/`sh`): the rest of the word/line must be
resident before a partial write, or the un-written bytes would later be flushed as
garbage.

### Miss-handling FSM

A miss may move a whole line **out** (eviction) and a whole line **in** (fill).
Each word transfer takes a cycle on the SRAM and on main memory, so the burst is
an explicit five-state machine:

```
        ┌──────┐  miss & dirty victim   ┌────────────┐
        │ IDLE │ ──────────────────────▶│ EVICT_READ │ ─┐ read victim word
        │      │                         └────────────┘  │ from cache SRAM
        │ hits │  miss & clean victim          │         ▼
        │served│ ───────────────┐              ┌────────────┐
        └──────┘                │              │EVICT_WRITE │ write it to
           ▲                    │              └────────────┘ main memory
           │                    ▼                    │ (loop until flushed)
           │             ┌────────────┐               ▼
           │             │  FILL_REQ  │◀──────────────┘
           │             └────────────┘ ─┐ request word from main memory
           │                    │         ▼
           │             ┌────────────┐
           └─────────────│ FILL_WRITE │ store word into cache SRAM
              last word  └────────────┘  (loop until filled)
```

When the burst finishes, the line is resident, valid and (just-filled) clean, and
the still-stalled access **replays as an ordinary hit** through the same datapath
as the write-through controller — which is also what performs the store and sets
the dirty bit for a write miss.

### Parameterizable data-bus widths

`DMWBCtrl` adds two width parameters on top of the structural ones:

| Parameter    | Default | Scope                                            |
|--------------|---------|--------------------------------------------------|
| `DATA_WIDTH` | 32      | core ↔ controller, and the cache SRAM word       |
| `MEM_WIDTH`  | 32      | controller ↔ main memory (transfer / **beat**)   |

`DATA_WIDTH` must be an integer multiple of `MEM_WIDTH` (they are equal in the
scalar 32/32 default). A line is moved in `BEATS_PER_LINE = LINE_BYTES /
(MEM_WIDTH/8)` beats; when the cache word is wider than a beat, `BEATS_PER_WORD =
DATA_WIDTH / MEM_WIDTH` beats fill it, each landing in its own byte **lane**:

```
                 DATA_WIDTH cache word (e.g. 64b)
        ┌───────────────────────┬───────────────────────┐
        │   beat 1 (MEM_WIDTH)  │   beat 0 (MEM_WIDTH)   │
        └───────────────────────┴───────────────────────┘
```

This lets a wide master — e.g. the RS5 **vector unit reading/writing a whole
cache line in one access** — share the same controller, and lets the
controller↔memory bus be widened to fill a line in fewer beats. For the default
32/32 the lane logic collapses to plain full-width transfers, so the generated
logic is identical to the un-parameterized version.

---

## 5. Integration notes & assumptions

* **Hit latency:** reads return data the cycle after the address is presented
  (standard synchronous SRAM timing); the surrounding core/pipeline must account
  for this one-cycle latency. `busy_o` is only for misses.
* **Stable requests under `busy_o`:** the core must hold its request stable while
  `busy_o` is high; the controllers also register the missing address internally
  so a fill/eviction is immune to spurious input changes.
* **Cache SRAM:** assumed fixed 1-cycle latency, never stalls.
* **Single-port sharing:** the controllers drive one `cache_*` and one `mem_*`
  port; instruction and data traffic can share main memory through external
  arbitration (see the RS5 testbenches).
* **No flush / coherence primitive:** there is currently no `fence`/flush input to
  force dirty lines back to memory. A consumer that reads memory behind the cache
  (DMA, a memory-dump signature check, modifying instruction memory) must account
  for still-dirty data — e.g. the RS5 RISCOF testbench reads the signature through
  a cache-coherent view.

---

## 6. Files

| File               | Description                                            |
|--------------------|--------------------------------------------------------|
| `rtl/DMWTCtrl.sv`  | Direct-mapped write-through controller                 |
| `rtl/DMWBCtrl.sv`  | Direct-mapped write-back controller (parameterizable)  |

Used by RS5 in `sim/testbench.sv` (Verilator) and `riscof/riscof_tb.sv` (RISCOF),
with the cache SRAM and main memory provided by `sim/RAM_mem.sv`.

---

*GAPH — Hardware Design Support Group · PUCRS — Pontifical Catholic University of
Rio Grande do Sul.*
