# RoboticHand

## Run
### CMD
```console
> ./run.sh --SendPos --Inverse --Generic
```
- ```--Inverse```: Launch SOFA isntance with inverse solver
- ```--Generic```: Launch SOFA instance with Gerneric solver
- ```--SendPos```: Launch python script to control robot inside SOFA

### Python
```console
> python runSofaScript.py -g -c --arm750
```
- ```-g, --generic```: Launch SOFA instance with inverse solver (without, launch with inverse solver)
- ```-c, --cube```: Add cube to the scene
- ```--hand```: Add hand to the scene (attached to the arm if option ```--arm750```is added)
- ```--arm750```: Add arm to the scene
- ```--sm```: Use a SharedMemory to control the simualation

## Control
- (```W```, ```S```), (```Q```, ```D```), and (```shift```, ```alt```) control respectively the X, Y, and Z of the robot target point with the inverse solver
- (```UP```, ```DOWN```), (```LEFT```, ```RIGHT```), and (```PAGE_UP```, ```PAGE_DOWN```) control respectively the X, Y, and Z of the index target point with the inverse solver
- ```1```, ```2```, ```3```, ```4```, ```5```, ```6```: Control respectively each axis of the robot with the generic solver

## dependencies
- SharedMemory (will be isntalled by IRONBark):
```console
> pip install SharedMemory
```
- IRONBark:
```console
> pip install IRONBark
```
- pynput:
```console
> pip install pynput
```