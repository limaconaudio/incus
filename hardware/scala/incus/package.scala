import spinal.core.SpinalConfig

package object incus {
  def wrap[T](body : => T) = body
  def SpinalRtlConfig = SpinalConfig(targetDirectory = "hardware/netlist")
}
