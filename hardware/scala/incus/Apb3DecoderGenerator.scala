package incus

import spinal.core.{Area, assert, log2Up}
import spinal.lib.bus.amba3.apb.{Apb3, Apb3Config, Apb3Decoder}
import spinal.lib.bus.misc.SizeMapping
import spinal.lib.generator.{Generator, Handle, MemoryConnection, SimpleBus}

import scala.collection.mutable.ArrayBuffer


case class Apb3DecoderGenerator() extends Generator {

  case class SlaveModel(slave: Handle[Apb3], config : Handle[Apb3Config], address: BigInt) {
    configGenerator.dependencies += config
    def mapping = SizeMapping(address, (BigInt(1)) << config.addressWidth)
  }

  val models = ArrayBuffer[SlaveModel]()

  val configGenerator = new Generator{val dummy = 0}
  val inputConfig = configGenerator produce Apb3Config(
    addressWidth = log2Up(models.map(m => m.mapping.end + 1).max),
    dataWidth = models.head.config.dataWidth
  )

  inputConfig produce(tags += new SimpleBus(input, BigInt(1) << inputConfig.addressWidth))

  val input = inputConfig.produce{
    for (m <- models) assert(m.config.dataWidth == inputConfig.dataWidth)
    Apb3(inputConfig)
  }


  def addSlave(slave: Handle[Apb3], address: BigInt): Unit = addSlave(slave, slave.produce(slave.config), address)

  def addSlave(slave: Handle[Apb3], config : Handle[Apb3Config], address: BigInt): Unit = {
    dependencies += slave
    models += SlaveModel(slave, config, address)
    tags += new MemoryConnection(input, slave, address)
  }

  dependencies += input
  val logic = add task new Area {

    val decoder = Apb3Decoder(
      master = input,
      slaves = models.map(m => (m.slave.get, m.mapping))
    )
  }
}
