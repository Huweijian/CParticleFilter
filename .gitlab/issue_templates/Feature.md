## 需求和目标

(提出具体的需求点和设计指标)
- 空旷场地定位误差降低到3米内。
- 平均单次检出时间降低到200ms以内
- ...

## 算法流程

(简单介绍算法流程)
1. 采集GPS数据，包括经纬度，精度，搜星数量，参考[这个][1]。
2. 利用似然模型改变上一个状态的权重。
3. 根据新的权重重新估计当前状态的最优状态。


## 参考文献

(论文建议保留文献名称，网页则可以直接在正文中添加引用)
> 1. John S. Strong localization of photons in certain disordered dielectric superlattices[J]. Physical review letters, 1987, 58(23): 2486.
> 2. Abrahams E, Anderson P W, Licciardello D C, et al. Scaling theory of localization: Absence of quantum diffusion in two dimensions[J]. Physical Review Letters, 1979, 42(10): 673.

[1]: https://blog.csdn.net/q1370992706/article/details/79950542


