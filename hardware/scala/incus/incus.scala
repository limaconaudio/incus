package incus

import vexriscv.plugin._
import vexriscv._
import vexriscv.ip.{DataCacheConfig, InstructionCacheConfig}
import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.amba4.axi._
import spinal.lib.com.uart.{Apb3UartCtrl, Uart, UartCtrlGenerics, UartCtrlMemoryMappedConfig}
import spinal.lib.io.{Apb3Gpio2, Gpio, TriStateArray}
import spinal.lib.misc.HexTools

import scala.collection.mutable.ArrayBuffer

case class IncusConfig(axiFrequency : HertzNumber,
                       onChipRamSize : BigInt,
                       gpioA : Gpio.Parameter,
                       cpuPlugins : ArrayBuffer[Plugin[VexRiscv]],
                       uartCtrlConfig : UartCtrlMemoryMappedConfig)

object IncusConfig{

  def default = {
    val config = IncusConfig(
      axiFrequency = 100 MHz,
      onChipRamSize  = 8 kB,
      gpioA = Gpio.Parameter(
        width = 4
      ),
      uartCtrlConfig = UartCtrlMemoryMappedConfig(
        uartCtrlConfig = UartCtrlGenerics(
          dataWidthMax      = 8,
          clockDividerWidth = 20,
          preSamplingSize   = 1,
          samplingSize      = 5,
          postSamplingSize  = 2
        ),
        txFifoDepth = 16,
        rxFifoDepth = 16
      ),
      cpuPlugins = ArrayBuffer(
        new PcManagerSimplePlugin(0x80000000l, false),
        new IBusCachedPlugin(
          resetVector = 0x80000000l,
          prediction = STATIC,
          config = InstructionCacheConfig(
            cacheSize = 4096,
            bytePerLine =32,
            wayCount = 1,
            addressWidth = 32,
            cpuDataWidth = 32,
            memDataWidth = 32,
            catchIllegalAccess = true,
            catchAccessFault = true,
            catchMemoryTranslationMiss = true,
            asyncTagMemory = false,
            twoCycleRam = true,
            twoCycleCache = true
          )
        ),
        new DBusCachedPlugin(
          config = new DataCacheConfig(
            cacheSize         = 4096,
            bytePerLine       = 32,
            wayCount          = 1,
            addressWidth      = 32,
            cpuDataWidth      = 32,
            memDataWidth      = 32,
            catchAccessError  = true,
            catchIllegal      = true,
            catchUnaligned    = true,
            catchMemoryTranslationMiss = true
          ),
          memoryTranslatorPortConfig = null
        ),
        new StaticMemoryTranslatorPlugin(
          ioRange      = _(31 downto 28) === 0xF
        ),
        new DecoderSimplePlugin(
          catchIllegalInstruction = true
        ),
        new RegFilePlugin(
          regFileReadyKind = plugin.SYNC,
          zeroBoot = false
        ),
        new IntAluPlugin,
        new SrcPlugin(
          separatedAddSub = false,
          executeInsertion = true
        ),
        new FullBarrelShifterPlugin,
        new MulPlugin,
        new DivPlugin,
        new HazardSimplePlugin(
          bypassExecute           = true,
          bypassMemory            = true,
          bypassWriteBack         = true,
          bypassWriteBackBuffer   = true,
          pessimisticUseSrc       = false,
          pessimisticWriteRegFile = false,
          pessimisticAddressMatch = false
        ),
        new BranchPlugin(
          earlyBranch = false,
          catchAddressMisaligned = true
        ),
        new CsrPlugin(
          config = CsrPluginConfig(
            catchIllegalAccess = false,
            mvendorid      = null,
            marchid        = null,
            mimpid         = null,
            mhartid        = null,
            misaExtensionsInit = 66,
            misaAccess     = CsrAccess.NONE,
            mtvecAccess    = CsrAccess.NONE,
            mtvecInit      = 0x80000020l,
            mepcAccess     = CsrAccess.READ_WRITE,
            mscratchGen    = false,
            mcauseAccess   = CsrAccess.READ_ONLY,
            mbadaddrAccess = CsrAccess.READ_ONLY,
            mcycleAccess   = CsrAccess.NONE,
            minstretAccess = CsrAccess.NONE,
            ecallGen       = false,
            wfiGenAsWait         = false,
            ucycleAccess   = CsrAccess.NONE
          )
        ),
        new YamlPlugin("cpu0.yaml")
      )
    )
    config
  }
}



class Incus(config: IncusConfig) extends Component{

  //Legacy constructor
  def this(axiFrequency: HertzNumber) {
    this(IncusConfig.default.copy(axiFrequency = axiFrequency))
  }

  import config._

  val io = new Bundle{
    //Clocks / reset
    val asyncReset = in Bool
    val axiClk     = in Bool

    //Peripherals IO
    val gpioA         = master(TriStateArray(4 bits))
    val uart          = master(Uart())
    val coreInterrupt = in Bool
  }

  val resetCtrlClockDomain = ClockDomain(
    clock = io.axiClk,
    config = ClockDomainConfig(
      resetKind = BOOT
    )
  )

  val resetCtrl = new ClockingArea(resetCtrlClockDomain) {
    val systemResetUnbuffered  = False

    //Implement an counter to keep the reset axiResetOrder high 64 cycles
    // Also this counter will automaticly do a reset when the system boot.
    val systemResetCounter = Reg(UInt(6 bits)) init(0)
    when(systemResetCounter =/= U(systemResetCounter.range -> true)){
      systemResetCounter := systemResetCounter + 1
      systemResetUnbuffered := True
    }
    when(BufferCC(io.asyncReset)){
      systemResetCounter := 0
    }

    //Create all reset used later in the design
    val systemReset  = RegNext(systemResetUnbuffered)
    val axiReset     = RegNext(systemResetUnbuffered)
  }

  val axiClockDomain = ClockDomain(
    clock = io.axiClk,
    reset = resetCtrl.axiReset,
    frequency = FixedFrequency(axiFrequency)
  )

  val axi = new ClockingArea(axiClockDomain) {
    val ram = Axi4SharedOnChipRam(
      dataWidth = 32,
      byteCount = onChipRamSize,
      idWidth = 4
    )

    val apbBridge = Axi4SharedToApb3Bridge(
      addressWidth = 20,
      dataWidth    = 32,
      idWidth      = 4
    )

    val gpioACtrl = Apb3Gpio2(config.gpioA)
    val machineTimer = MachineTimer()

    val uartCtrl = Apb3UartCtrl(uartCtrlConfig)

    val core = new Area{
      val config = VexRiscvConfig(
        plugins = cpuPlugins
      )

      val cpu = new VexRiscv(config)
      var iBus : Axi4ReadOnly = null
      var dBus : Axi4Shared = null
      for(plugin <- config.plugins) plugin match{
        case plugin : IBusCachedPlugin => iBus = plugin.iBus.toAxi4ReadOnly()
        case plugin : DBusCachedPlugin => dBus = plugin.dBus.toAxi4Shared(true)
        case plugin : CsrPlugin        => {
          plugin.externalInterrupt := BufferCC(io.coreInterrupt)
//          plugin.timerInterrupt := machineTimer.io.mTimeInterrupt
          plugin.timerInterrupt := False
        }
        case _ =>
      }
    }


    val axiCrossbar = Axi4CrossbarFactory()

    axiCrossbar.addSlaves(
      ram.io.axi       -> (0x80000000L,   onChipRamSize),
      apbBridge.io.axi -> (0xF0000000L,   1 MB)
    )

    axiCrossbar.addConnections(
      core.iBus       -> List(ram.io.axi),
      core.dBus       -> List(ram.io.axi, apbBridge.io.axi)
    )

    axiCrossbar.addPipelining(apbBridge.io.axi)((crossbar,bridge) => {
      crossbar.sharedCmd.halfPipe() >> bridge.sharedCmd
      crossbar.writeData.halfPipe() >> bridge.writeData
      crossbar.writeRsp             << bridge.writeRsp
      crossbar.readRsp              << bridge.readRsp
    })

    axiCrossbar.addPipelining(ram.io.axi)((crossbar,ctrl) => {
      crossbar.sharedCmd.halfPipe()  >>  ctrl.sharedCmd
      crossbar.writeData            >/-> ctrl.writeData
      crossbar.writeRsp              <<  ctrl.writeRsp
      crossbar.readRsp               <<  ctrl.readRsp
    })

    axiCrossbar.addPipelining(core.dBus)((cpu,crossbar) => {
      cpu.sharedCmd             >>  crossbar.sharedCmd
      cpu.writeData             >>  crossbar.writeData
      cpu.writeRsp              <<  crossbar.writeRsp
      cpu.readRsp               <-< crossbar.readRsp //Data cache directly use read responses without buffering, so pipeline it for FMax
    })

    axiCrossbar.build()


    val apbDecoder = Apb3Decoder(
      master = apbBridge.io.apb,
      slaves = List(
        gpioACtrl.io.bus -> (0x00000, 4 kB),
	machineTimer.io.bus -> (0x08000, 4 kB),
        uartCtrl.io.apb  -> (0x10000, 4 kB)
      )
    )
  }

  io.gpioA          <> axi.gpioACtrl.io.gpio
  io.uart           <> axi.uartCtrl.io.uart
}

object Incus{
  def main(args: Array[String]) {
    val config = SpinalConfig()
    config.generateVerilog({
      val toplevel = new Incus(IncusConfig.default)
      toplevel
    })
  }
}

object IncusWithMemoryInit{
  def main(args: Array[String]) {
    val config = SpinalConfig()
    config.generateVerilog({
      val toplevel = new Incus(IncusConfig.default)
      HexTools.initRam(toplevel.axi.ram.ram, "software/test/resource/uart.hex", 0x80000000l)
      toplevel
    })
  }
}

case class IncusArty(config : IncusConfig) extends Component{
  val io = new Bundle {
    val CLK = in Bool()
    val TXD = out Bool()
    val RXD = in Bool()
    val LED = out Bits(4 bits)
  }.setName("")

  val soc = new Incus(config)
  HexTools.initRam(soc.axi.ram.ram, "software/c/timer/build/timer.hex", 0x80000000l)

  soc.io.axiClk   <> io.CLK
  soc.io.asyncReset <> False

  soc.io.uart.txd <> io.TXD
  soc.io.uart.rxd <> io.RXD

  soc.io.gpioA.read := B"0000"
  io.LED := soc.io.gpioA.write

  soc.io.coreInterrupt := False
}

//Scala main used to generate the toplevel
object IncusArty{
  def main(args: Array[String]) {
    val config = SpinalConfig()
    config.generateVerilog({
      val toplevel = IncusArty(IncusConfig.default)
      toplevel
    })
  }
}
