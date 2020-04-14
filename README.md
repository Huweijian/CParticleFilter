C++粒子滤波，模仿MATLAB的robotics.ParticleFilter实现

注：相比MATLAB 粒子滤波暂未实现的功能：
- 循环变量（比如角度，在[0, 360]范围内循环）
- 重采样方法（目前只实现了multinomial）
- 状态估计方法（目前只实现了均值）

#### 使用方法
参考/Demo 文件夹中的例子  
详细API可以参考 [MATLAB robotics.ParticleFilter][1]

#### 依赖
[Eigen3](http://eigen.tuxfamily.org)

#### 参考文档
1. [MATLAB粒子滤波 **参数选择**](http://ww2.mathworks.cn/help/robotics/ug/particle-filter-parameters.html)
2. [MATLAB粒子滤波 **工作流程**](http://ww2.mathworks.cn/help/robotics/ug/particle-filter-workflow.html)


[1]: https://ww2.mathworks.cn/help/robotics/ref/robotics.particlefilter-class.html
