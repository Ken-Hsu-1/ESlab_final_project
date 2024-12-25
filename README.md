# GestureSync: STM32-Controlled YouTube Player

This is a project using Gesture Recognition with LSM6DSL in STM32L4 and communicate Python code via WIFI to control youtube

More information can be find in [doc/report.pdf](https://github.com/Ken-Hsu-1/ESlab_final_project/blob/main/doc/report.pdf) and [demo video playlist](https://youtube.com/playlist?list=PLpyxc1voi02qkzzVdoig6L7b6ILT59Pj0&feature=shared)

(Noticed that you should rewrite the WIFI information in main.c before compiling STM32L4 code)

     ├──doc
     │    ├──presentation.pdf -> project introduction in ppt fromat
     │    ├──report.pdf -> project report with all informations and details 
     ├──python
     │    ├──AbsV.py -> Python code with absolute volume control
     │    ├──Revv.py -> Python code with relative volume control
     ├──stm32
     │    ├──project/ -> the whole project in STM32L4 
     │    ├──main.c   -> the core of this project , is the same with stm32/project/Core/Src/main.c

     
