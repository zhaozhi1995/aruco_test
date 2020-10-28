## Aruco Marker ROS demo
调用opencv实现 Aruco Marker的生成，检测及定位。
代码地址：[https://gitee.com/nie_xun/aruco_test.git](htts://gitee.com/nie_xun/aruco_test.git)

## 使用
1. 源码下载编译
```bash
cd catkin_ws/src
git clone https://gitee.com/nie_xun/aruco_test.git
cd ..
catkin_make
```
2. Aruco Marker生成

```bash
roslaunch aruco_test aruco_marker_generator.launch
```
aruco_marker_generator.launch 中，根据需要设置以下参数。
```xml
<?xml version="1.0"?>
<launch>
  <env name="ROSCONSOLE_CONFIG_FILE" value="$(find aruco_test)/config/log.conf"/>
  <node pkg="aruco_test" type="aruco_marker_generator" name="aruco_marker_generator" output="screen" >
    <param name="marker_save_dir" type="string" value="$(find aruco_test)/data/"/>
    <param name="marker_id" type="int" value="26"/>
    <param name="rowXcolumn" type="int" value="5"/>
    <param name="marker_size" type="int" value="250"/>
    <!--<param name="marker_DICT" type="string" value="DICT_ARUCO_ORIGINAL"/>-->
  </node>
</launch>
```
3. Aruco Marker检测

```bash
#首先启动相机节点，在启动aruco_test
roslaunch aruco_test aruco_test.launch
```
结果如下图：目前不能检测DICT_ARUCO_ORIGINAL
![在这里插入图片描述](https://img-blog.csdnimg.cn/20201028144247444.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MTQ2OTI3Mg==,size_16,color_FFFFFF,t_70#pic_center)

