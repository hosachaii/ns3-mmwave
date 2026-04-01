#!/bin/bash

# clear old results
rm -f results.csv

TCPs=("TcpMrvhs" "TcpHighSpeed" "TcpNewReno")
ERRORS=(1e-6 1e-5 1e-4 1e-3)

for tcp in "${TCPs[@]}"
do
  for err in "${ERRORS[@]}"
  do
    echo "Running $tcp with error $err"

    ./ns3 run "scratch/mrvhs-simulation \
        --tcpType=$tcp \
        --errorRate=$err \
        --delay=40 \
        --dataRate=1Gbps"
  done
done

echo "Done. Results saved in results.csv"
