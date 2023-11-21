### 滑台程序

- msg修改

  1. 将原来的slider.msg更新，本消息类型后续只作为带参数的控制命令传递；

     ![image-20231121180758222](C:\Users\吕锐\AppData\Roaming\Typora\typora-user-images\image-20231121180758222.png)

  2. 新增operation.msg ，本消息类型只有一个”命令字“，为无参数的控制命令传递；

- scripts修改：（node.py的修改）

  1. 将使用两种不同消息类型的控制命令分开，用两种话题接受命令；
  2. 增加了检查 X 轴的 微步细分 和 pps 值是否一致；

- src修改：

  1. 补充完善了后续功能；

