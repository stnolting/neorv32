version 1
test wdt_access
tb tb_wdt
args -gCFG=1
image none
cycles 203
window 1 8
stdout 6498bdbaddc516c6 1
global 0:424734,10:2d09dc,11:6aeb38,12:e5a8d7,13:a24a33,14:eed2ba,15:655822,116:5db935,117:1a5bd1,128:bfc2d8,129:f8203c,140:3c758f,141:7b976b,142:f962cc,143:6a5f17,144:ad3799,145:ead57d,166:ec877a,167:143c17,168:f0e6cb,169:b7042f,170:d46f12,171:938df6,192:a245e1,193:5afe8c,194:7a5659,195:3db4bd,201:d34508,202:b34a53
signal bus_rsp_o.ack 29 0a7c8d7c668de18c 0:b1beb0,1:c6aa2f,14:f34882,16:a713d3,17:13af4c,18:b87306,20:e88111,21:57514a,24:ba4ef1,25:15f5e7
signal bus_rsp_o.err 1 59b70cfcb99bce28 0:b1beb0
signal bus_rsp_o.data 11 af79f74f0d572ca8 0:a7b22a,1:a88e35,17:49f661,20:1700a9,24:071d1a,25:23d3ec
signal rstn_o 1 b171e99a0ef8f300 0:aa8ff1

version 1
test wdt_debug_reset
tb tb_wdt
args -gCFG=2
image none
cycles 148
window 1 8
stdout d7a83ffcdcf1a730 1
global 0:424734,10:2d09dc,11:6aeb38,12:e5a8d7,13:a24a33,14:eed2ba,15:655822,116:5db935,117:1a5bd1,128:bfc2d8,129:f8203c,146:ac6566,147:2a313a
signal bus_rsp_o.ack 13 988b8c9353393f8b 0:b1beb0,1:c6aa2f,14:f34882,16:a713d3,18:446bb0
signal bus_rsp_o.err 1 59b70cfcb99bce28 0:b1beb0
signal bus_rsp_o.data 5 ffb59c37d1dde6a3 0:a7b22a,1:a88e35,18:176ddd
signal rstn_o 1 b171e99a0ef8f300 0:aa8ff1

version 1
test wdt_timeout
tb tb_wdt
args -gCFG=0
image none
cycles 20144
window 64 1024
stdout 2762f2aa9f07912c 1
global 0:b31cfb,1:6fff1a,2:8f0089,314:cbe079
signal bus_rsp_o.ack 15 2e4c250a98f30174 0:057d37,19:6019f5
signal bus_rsp_o.err 1 59b70cfcb99bce28 0:b1beb0
signal bus_rsp_o.data 5 ec2d3911831a04b1 0:4dc880,19:603aaa
signal rstn_o 1 b171e99a0ef8f300 0:aa8ff1
