#
# This file is part of LiteX.
#
# Copyright (c) 2020-2021 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Direct Memory Access (DMA) reader and writer modules."""

from migen import *

from litex.gen import *
from litex.gen.common import reverse_bytes

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream
from litex.soc.interconnect import wishbone

# Helpers ------------------------------------------------------------------------------------------

def format_bytes(s, endianness):
    return {"big": s, "little": reverse_bytes(s)}[endianness]

# WishboneDMAReader --------------------------------------------------------------------------------

class WishboneDMAReader(LiteXModule):
    """Read data from Wishbone MMAP memory.

    For every address written to the sink, one word will be produced on the source.

    Parameters
    ----------
    bus : bus
        Wishbone bus of the SoC to read from.

    Attributes
    ----------
    sink : Record("address")
        Sink for MMAP addresses to be read.

    source : Record("data")
        Source for MMAP word results from reading.
    """
    def __init__(self, bus, endianness="little", fifo_depth=16, with_csr=False):
        assert isinstance(bus, wishbone.Interface)
        assert "r" in bus.mode
        self.bus    = bus
        self.sink   = sink   = stream.Endpoint([("address", bus.adr_width, ("last", 1))])
        self.source = source = stream.Endpoint([("data",    bus.data_width)])

        # # #

        # FIFO..
        self.fifo = fifo = stream.SyncFIFO([("data", bus.data_width)], depth=fifo_depth)

        # Reads -> FIFO.
        self.comb += [
            bus.stb.eq(sink.valid & fifo.sink.ready),
            bus.cyc.eq(sink.valid & fifo.sink.ready),
            bus.we.eq(0),
            bus.sel.eq(2**(bus.data_width//8)-1),
            bus.adr.eq(sink.address),
            fifo.sink.last.eq(sink.last),
            fifo.sink.data.eq(format_bytes(bus.dat_r, endianness)),
            If(bus.stb & bus.ack,
                sink.ready.eq(1),
                fifo.sink.valid.eq(1),
            ),
        ]

        # FIFO -> Output.
        self.comb += fifo.source.connect(source)

        # CSRs.
        if with_csr:
            self.add_csr()

    def add_ctrl(self, default_base=0, default_length=0, default_enable=0, default_loop=0):
        self.base   = Signal(64, reset=default_base)
        self.length = Signal(32, reset=default_length)
        self.enable = Signal(reset=default_enable)
        self.done   = Signal()
        self.loop   = Signal(reset=default_loop)
        self.offset = Signal(32)

        # # #

        shift   = log2_int(self.bus.data_width//8)
        base    = Signal(self.bus.adr_width)
        offset  = Signal(self.bus.adr_width)
        length  = Signal(self.bus.adr_width)
        self.comb += base.eq(self.base[shift:])
        self.comb += length.eq(self.length[shift:])

        self.comb += self.offset.eq(offset)

        self.fsm = fsm = ResetInserter()(FSM(reset_state="IDLE"))
        self.comb += fsm.reset.eq(~self.enable)
        fsm.act("IDLE",
            NextValue(offset, 0),
            NextState("RUN"),
        )
        fsm.act("RUN",
            self.sink.valid.eq(1),
            self.sink.last.eq(offset == (length - 1)),
            self.sink.address.eq(base + offset),
            If(self.sink.ready,
                NextValue(offset, offset + 1),
                If(self.sink.last,
                    If(self.loop,
                        NextValue(offset, 0)
                    ).Else(
                        NextState("DONE")
                    )
                )
            )
        )
        fsm.act("DONE", self.done.eq(1))

    def add_csr(self, default_base=0, default_length=0, default_enable=0, default_loop=0):
        if not hasattr(self, "base"):
            self.add_ctrl()
        self._base   = CSRStorage(64, reset=default_base)
        self._length = CSRStorage(32, reset=default_length)
        self._enable = CSRStorage(reset=default_enable)
        self._done   = CSRStatus()
        self._loop   = CSRStorage(reset=default_loop)
        self._offset = CSRStatus(32)

        # # #

        self.comb += [
            # Control.
            self.base.eq(self._base.storage),
            self.length.eq(self._length.storage),
            self.enable.eq(self._enable.storage),
            self.loop.eq(self._loop.storage),
            # Status.
            self._done.status.eq(self.done),
            self._offset.status.eq(self.offset),
        ]

# WishboneDMAWriter --------------------------------------------------------------------------------

class WishboneDMAWriter(LiteXModule):
    """Write data to Wishbone MMAP memory.

    Parameters
    ----------
    bus : bus
        Wishbone bus of the SoC to read from.

    Attributes
    ----------
    sink : Record("address", "data")
        Sink for MMAP addresses/datas to be written.
    """
    def __init__(self, bus, endianness="little", with_csr=False):
        assert isinstance(bus, wishbone.Interface)
        assert "w" in bus.mode
        self.bus  = bus
        self.sink = sink = stream.Endpoint([("address", bus.adr_width), ("data", bus.data_width)])

        # # #

        # Writes.
        data = Signal(bus.data_width)
        self.comb += [
            bus.stb.eq(sink.valid),
            bus.cyc.eq(sink.valid),
            bus.we.eq(1),
            bus.sel.eq(2**(bus.data_width//8)-1),
            bus.adr.eq(sink.address),
            bus.dat_w.eq(format_bytes(sink.data, endianness)),
            sink.ready.eq(bus.ack),
        ]

        # CSRs.
        if with_csr:
            self.add_csr()

    def add_ctrl(self, default_base=0, default_length=0, default_enable=0, default_loop=0, ready_on_idle=1):
        self._sink = self.sink
        self.sink  = stream.Endpoint([("data", self.bus.data_width)])

        self.base   = Signal(64, reset=default_base)
        self.length = Signal(32, reset=default_length)
        self.enable = Signal(reset=default_enable)
        self.done   = Signal()
        self.loop   = Signal(reset=default_loop)
        self.offset = Signal(32)

        # # #

        shift   = log2_int(self.bus.data_width//8)
        base    = Signal(self.bus.adr_width)
        offset  = Signal(self.bus.adr_width)
        length  = Signal(self.bus.adr_width)
        self.comb += base.eq(self.base[shift:])
        self.comb += length.eq(self.length[shift:])

        self.comb += self.offset.eq(offset)

        self.fsm = fsm = ResetInserter()(FSM(reset_state="IDLE"))
        self.comb += fsm.reset.eq(~self.enable)
        fsm.act("IDLE",
            self.sink.ready.eq(ready_on_idle),
            NextValue(offset, 0),
            NextState("RUN"),
        )
        fsm.act("RUN",
            self._sink.valid.eq(self.sink.valid),
            self._sink.last.eq(self.sink.last | (offset + 1 == length)),
            self._sink.address.eq(base + offset),
            self._sink.data.eq(self.sink.data),
            self.sink.ready.eq(self._sink.ready),
            If(self.sink.valid & self.sink.ready,
                NextValue(offset, offset + 1),
                If(self._sink.last,
                    If(self.loop,
                        NextValue(offset, 0)
                    ).Else(
                        NextState("DONE")
                    )
                )
            )
        )
        fsm.act("DONE", self.done.eq(1))

    def add_csr(self, default_base=0, default_length=0, default_enable=0, default_loop=0):
        if not hasattr(self, "base"):
            self.add_ctrl()
        self._base   = CSRStorage(64, reset=default_base)
        self._length = CSRStorage(32, reset=default_length)
        self._enable = CSRStorage(reset=default_enable)
        self._done   = CSRStatus()
        self._loop   = CSRStorage(reset=default_loop)
        self._offset = CSRStatus(32)

        # # #

        self.comb += [
            # Control.
            self.base.eq(self._base.storage),
            self.length.eq(self._length.storage),
            self.enable.eq(self._enable.storage),
            self.loop.eq(self._loop.storage),
            # Status.
            self._done.status.eq(self.done),
            self._offset.status.eq(self.offset),
        ]

def dma_desc_description():
    payload_layout = [
        ("address", 64),
        ("length",  32),
        ("status",  32)
    ]

    return stream.EndpointDescription(payload_layout)

DMA_DESC_STATUS_DONE  = 0x1
DMA_DESC_STATUS_ERROR = 0x2
DMA_DESC_STATUS_LAST  = 0x10

class WishboneDMADescriptorFetcher(LiteXModule):
    def __init__(self, bus, endianness="little"):
        assert isinstance(bus, wishbone.Interface)
        assert "r" in bus.mode
        assert (128 % bus.data_width) == 0

        self.bus   = bus
        self.depth = depth = 128 // bus.data_width

        self.sink  = sink  = stream.Endpoint([("address", bus.adr_width, ("last", 1))])
        self.desc  = desc  = stream.Endpoint(dma_desc_description())
        self.converter = converter = stream.Converter(bus.data_width, 128, reverse=True)
        self.fifo  = fifo  = stream.SyncFIFO(["data", 128], 1)

        self.base   = Signal(self.bus.adr_width)
        self.offset = Signal(self.bus.adr_width)
        self.enable = Signal()
        self.done   = Signal()

        # ADDRESS Geneartion FSM

        self.fsm = fsm = ResetInserter()(FSM(reset_state="IDLE"))
        self.comb += fsm.reset.eq(~self.enable)
        fsm.act("IDLE",
            NextValue(offset, 0),
            NextState("RUN"),
        )
        fsm.act("RUN",
            self.sink.valid.eq(1),
            self.sink.last.eq(offset == (depth - 1)),
            self.sink.address.eq(base + offset),
            If(self.sink.ready,
                NextValue(offset, offset + 1),
                If(self.sink.last,
                    NextState("DONE")
                )
            )
        )
        fsm.act("DONE", self.done.eq(1))

        # BUS -> CONVERTER.
        self.comb += [
            bus.adr.eq(sink.address),

            bus.we.eq(0),
            bus.sel.eq(2**(bus.data_width//8)-1),

            bus.cyc.eq(sink.valid & converter.sink.ready),
            bus.stb.eq(sink.valid & converter.sink.ready),

            converter.sink.last.eq(sink.last),
            converter.sink.data.eq(format_bytes(bus.dat_r, endianness)),

            If(bus.stb & bus.ack,
                sink.ready.eq(1),
                converter.sink.valid.eq(1),
            ),

            converter.source.connect(fifo.sink),

            self.desc.valid.eq(fifo.source.valid),
            self.desc.ready.eq(fifo.source.ready),
            self.desc.address.eq(fifo.source.data[63:0]),
            self.desc.length.eq(fifo.source.data[95:64]),
            self.desc.status.eq(fifo.source.data[127:96]),
        ]

class WishboneDMAStatusUpdater(LiteXModule):
    def __init__(self, bus, endianness="little"):
        assert isinstance(bus, wishbone.Interface)
        assert "w" in bus.mode

        self.base    = Signal(self.bus.adr_width)
        self.data    = Signal(self.bus.data_width)
        self.enable  = Signal()
        self.done    = Signal()

        # Writes.
        self.comb += [
            bus.we.eq(1),
            bus.sel.eq(2**(bus.data_width//8)-1),
            bus.adr.eq(self.base),
            bus.dat_w.eq(format_bytes(self.data, endianness)),
        ]

        self.fsm = fsm = ResetInserter()(FSM(reset_state="IDLE"))
        self.comb += fsm.reset.eq(~self.enable)
        fsm.act("IDLE",
            NextState("RUN"),
        )
        fsm.act("RUN",
            bus.stb.eq(1),
            bus.cyc.eq(1),
            If(bus.ack,
                NextState("DONE")
            )
        )
        fsm.act("DONE", self.done.eq(1))

class WishboneDescDMAController(LiteXModule):
    def __init__(self, dma, updater_bus, endianness="little", with_csr=False):
        assert isinstance(fetch_bus, wishbone.Interface)
        assert isinstance(updater_bus, wishbone.Interface)
        assert "r" in fetch_bus.mode
        assert "w" in updater_bus.mode
        assert isinstance(dma, WishboneDMAReader) or isinstance(dma, WishboneDMAWriter)

        self.fetch_bus = fetch_bus
        self.updater_bus = updater_bus

        self.fetcher = WishboneDMADescriptorFetcher(fetch_bus, endianness)
        self.updater = WishboneDMAStatusUpdater(updater_bus, endianness)
        self.dma = dma
        self.dma.add_ctrl()

        self.comb += [
            self.dma.base.eq(self.fetcher.desc.address),
            self.dma.length.eq(self.fetcher.desc.length),
            self.dma.offset.eq(0),
            self.dma.loop.eq(0),
        ]

        update_data = Signal(32)
        update_data.eq(self.fetcher.desc.status | DMA_DESC_STATUS_DONE)
        if updater_bus.data_width == 32:
            self.comb += [
                self.updater.base.eq(self.fetcher.desc.address + 0xc),
                self.updater.length.eq(4),
                self.updater.data.eq(update_data)
            ]
        else:
            self.comb += [
                self.updater.base.eq(self.fetcher.desc.address + 0x8),
                self.updater.length.eq(8),
                self.updater.data.eq(Cat(self.fetcher.desc.length, update_data))
            ]


        # CSRs.
        if with_csr:
            self.add_csr()

    def add_ctrl(self, default_base=0, default_length=0, default_enable=0, default_loop=0, ready_on_idle=1):
        self.base   = Signal(64, reset=default_base)
        self.enable = Signal(reset=default_enable)
        self.single = Signal()
        self.done   = Signal()

        self.comb += self.fetcher.base.eq(self.base)

        # status data

        self.stats_port = stream.Endpoint([("length", "data")])

        # TODO: next package check

        self.fsm = fsm = ResetInserter()(FSM(reset_state="IDLE"))
        self.comb += fsm.reset.eq(~self.enable)
        fsm.act("IDLE",
            self.done.eq(0),
            self.fetcher.desc.ready.eq(ready_on_idle),
            NextState("RUN"),
        )
        fsm.act("RUN",
            # FETCH
            self.fetcher.enable.eq(1),
            IF(self.fetcher.done,
                self.dma.enable.eq(1),
            ),
            # DMA
            If(self.dma.done,
                self.updater.enable.eq(1),
            ),
            IF(self.updater.done,
                NextState("UPDATE"),
            ),
        )
        fsm.act("UPDATE",
            self.fetcher.enable.eq(0),
            self.dma.enable.eq(0),
            self.updater.enable.eq(0),
            self.single.eq(1),
            IF((self.fetcher.desc.status & DMA_DESC_STATUS_LAST) == DMA_DESC_STATUS_LAST,
                NextState(DONE),
            ).Else(
                NextValue(self.fetcher.base, self.fetcher.base + 0x10),
                NextState("FETCH"),
            )
        )
        fsm.act("DONE",
            self.done.eq(1),
        )

    def add_csr():
        if not hasattr(self, "base"):
            self.add_ctrl()
        self._base   = CSRStorage(64, reset=default_base)
        self._enable = CSRStorage(reset=default_enable)
        self._done   = CSRStatus(2)

        self.comb += [
            # Control.
            self.base.eq(self._base.storage),
            self.enable.eq(self._enable.storage),

            # Status.
            self._done.status(0).eq(self.single),
            self._done.status(1).eq(self.done),
        ]


# WishboneDescDMAWriter --------------------------------------------------------------------------------

class WishboneDescDMAWriter(WishboneDescDMAController):
    def __init__(self, dma_bus, fetch_bus, updater_bus, endianness="little", with_csr=False):
        assert isinstance(dma_bus, wishbone.Interface)
        assert "w" in dma_bus.mode
        self.dma_bus = dma_bus

        dma = WishboneDMAWriter(dma_bus, endianness, with_csr=False)
        super().__init__(dma, updater_bus, endianness, with_csr)

        self.sink = sink = stream.Endpoint([("data", dma_bus.data_width)])
        self.comb += dma.sink.connect(sink)


# WishboneDescDMAWriter --------------------------------------------------------------------------------

class WishboneDescDMAReader(WishboneDescDMAController):
    def __init__(self, dma_bus, fetch_bus, updater_bus, endianness="little", with_csr=False):
        assert isinstance(dma_bus, wishbone.Interface)
        assert "r" in dma_bus.mode
        self.dma_bus = dma_bus

        dma = WishboneDMAReader(dma_bus, endianness, with_csr=False)

        super().__init__(dma, updater_bus, endianness, with_csr)

        self.source = source = stream.Endpoint([("data", dma_bus.data_width)])
        self.comb += dma.source.connect(source)

