#!/bin/sh

sbt "runMain incus.IncusArty"

sleep 1

# Replace .bin with .mem in verilog
sed -i s/\.bin/\.mem/g IncusArty.v

rm -rf *.mem

# Generate .mem files
cp IncusArty.v_toplevel_soc_axi_ram_ram_symbol0.bin IncusArty.v_toplevel_soc_axi_ram_ram_symbol0.mem
cp IncusArty.v_toplevel_soc_axi_ram_ram_symbol1.bin IncusArty.v_toplevel_soc_axi_ram_ram_symbol1.mem
cp IncusArty.v_toplevel_soc_axi_ram_ram_symbol2.bin IncusArty.v_toplevel_soc_axi_ram_ram_symbol2.mem
cp IncusArty.v_toplevel_soc_axi_ram_ram_symbol3.bin IncusArty.v_toplevel_soc_axi_ram_ram_symbol3.mem
