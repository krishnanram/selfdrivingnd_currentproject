#!/bin/bash

echo "running ..."
N=20
dt=0.25
ref_v=60
ref_cte=0
ref_epsi=0
Lf=2.65

nactuator_limit=1.0e19
delta_limit=0.436332
acc_limit=1.0


echo "N="$N
echo "dt="$dt
echo "ref_v="$ref_v
echo "ref_cte="$ref_cte
echo "ref_epsi="$ref_epsi
echo "Lf="$Lf

echo "nactuator_limit="$nactuator_limit
echo "delta_limit="$delta_limit
echo "acc_limit="$acc_limit

../../build/mpc $N $dt $ref_v $ref_cte $ref_epsi $Lf $nactuator_limit $delta_limit $acc_limit
