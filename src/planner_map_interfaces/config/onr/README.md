9-12-2024

Sensor model changed from CMU to Arete MAX. The company, Arete did simulations on synthetic 
data. They simulated in perfect (MAX) visibility and (MIN) poor visibilty conditions. 

The simulations were over a wide range of vessel sizes. I chose the largest ones and assigned values 
to our PNR column for the ranges Arete gave. They did not provide NPR values. I did a POAB comparison
and saw coverage improvement when substituting MAX for our model. 

Last, our CMU sensor model is renamed. The MAX Arete model was copied and has the name 
"sensor_model_0.csv" now.. Likewise, "sensor_params_CMU.yaml" is the old yaml we used for the CMU
sensor_model_0.csv.

- mnclark
