#!/bin/bash

# disable reverse path filtering
sudo sysctl -w net.ipv4.conf.all.rp_filter=0
sudo sysctl -w net.ipv4.conf.<used_network_interface>.rp_filter=0

# increase buffer size
sudo sysctl -w net.core.rmem_max=10485760
sudo sysctl -w net.core.rmem_default=10485760

# enable jumbo packet
sudo ifconfig <used_network_interface> mtu 9000
