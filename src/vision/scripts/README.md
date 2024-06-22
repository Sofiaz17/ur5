Per far partire il nodo visione usa il comando

1. Fai partire gazebo

2. Segui le istruzioni qui sotto:

ros_ws => usa il tuo workspace ti catkin o ros


preset
```
. ~/ros_ws/devel/setup.bash
cd rosws
catkin_make
```

poi, per arrivare al file .py
```
roscd vision
cd scripts
```

e infine per lanciare il nodo:
```
python3 vision-node.py 
```
