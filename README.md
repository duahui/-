一、global_map
    1、生成障碍物信息及地图信息
    2、生成海浪信息
二、全局规划 -- grid_searcher包
    1、apollo里的无人车 Hybrid A*版本
    2、Fast-planner 里的无人机 Hybrid A*版本
    3、searcher_manager用于管理各方法类，方便扩展
三、轨迹优化-- optimizer包
    1、无人机版 minmium——snap优化
    2、Piecewise-jerk的速度规划，OSQP求解
    3、无人车版动静态障碍物下的非线性优化问题。参考apollo双重多态实现IPOPT。
    4、opt-manager管理各类方法，方便扩展。
四、总问题调用-- nav包
    各模块包生成类，方便对各模块进行协作
    

[https://github.com/duahui/-/assets/75663164/9e6de508-bc1c-4cd8-9a52-35311526c268](https://github.com/duahui/-/assets/75663164/34e1b3bd-9352-43ca-9553-178481752ae4)
