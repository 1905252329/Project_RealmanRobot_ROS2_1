# Ubuntu / ROS2 常用命令学习

```bash
. .venv/bin/activate && echo $PYTHONPATH && pip list | grep -i robot
```

这条命令由三个部分组成，通过 `&&` 运算符连接，表示依次执行：

1. `. .venv/bin/activate`
   - 这个命令用来激活Python虚拟环境
   - `.venv` 是虚拟环境的目录名
   - `bin/activate` 是激活脚本的路径
   - 使用 [.](javascript:void(0)) (source命令的简写) 来执行脚本，使环境变量在当前shell会话中生效
   - 激活后，后续的Python和pip命令将使用虚拟环境中的Python解释器和包
2. `echo $PYTHONPATH`
   - 输出当前PYTHONPATH环境变量的值
   - PYTHONPATH是Python用来寻找模块和包的路径列表
   - 当Python导入模块时，会在这个路径列表中查找
   - 这有助于理解Python解释器在哪里寻找包和模块
3. `pip list | grep -i robot`
   - `pip list` 列出当前环境中安装的所有Python包
   - `|` 是管道符，将前一个命令的输出传递给后一个命令作为输入
   - `grep -i robot` 在输出中搜索包含"robot"的行（不区分大小写）
   - 这用于查看环境中安装的与"robot"相关的包

整条命令的目的是检查在虚拟环境中：

- PYTHONPATH的设置情况
- 是否安装了与机器人相关的Python包及其版本

这对于调试Python包导入问题非常有用，特别是当您在不同的Python环境中有不同的包安装时

> 查看当前 PYTHONPATH 环境变量的值，在命令 echo $PYTHONPATH 后添加什么指令，可以让输出的路径按行显示，要求命令最少

可以使用以下命令在显示 PYTHONPATH 时让路径按行显示：

```bash
echo $PYTHONPATH | tr ':' '\n'
```

这个命令使用 `tr` 工具将路径分隔符 `:` 替换为换行符 `\n`，从而实现按行显示每个路径。

所以完整的命令将是：

```bash
. .venv/bin/activate && echo $PYTHONPATH | tr ':' '\n' && pip list | grep -i robot
```

这样就可以实现：

1. 激活虚拟环境
2. 按行显示 PYTHONPATH 中的每个路径
3. 显示安装的与 robot 相关的包