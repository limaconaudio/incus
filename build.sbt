lazy val root = (project in file(".")).
  settings(
    inThisBuild(List(
      organization := "com.github.spinalhdl",
      scalaVersion := "2.11.12",
      version      := "1.0.0"
    )),
    libraryDependencies ++= Seq(
      "org.scalatest" % "scalatest_2.11" % "2.2.1",
      "org.yaml" % "snakeyaml" % "1.8"
    ),
    name := "incus",
    scalaSource in Compile := baseDirectory.value / "hardware" / "scala"
  ).dependsOn(vexRiscv)

lazy val vexRiscv = RootProject(file("../VexRiscv"))

fork := true

connectInput in run := true
