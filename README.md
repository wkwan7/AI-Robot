# AI-Robot

2022-1-13
1. FetchGoodBot逻辑可用(但是走路不避障，需要手动移开障碍；需要先点一下画面再按键'B'或'C'以指定所取货物)
2. 摩擦力调参(worldinfo-contactproperties)，LoadGoodBot调参(double Grasp_dis_set[], double grasp_force_threshold)，更好抓住物体
3. 缩短传送带，调整传送带朝向
4. 两车均增加激光雷达，雷达使用自定义的proto(./protos/SickLms291.proto)
