"""
저자 main.py 를 그대로 실행한다.

  main.py 는 저장소에 없는 robot_model.acrobot 을 import 만 하고 쓰지 않는다
  (공개할 때 빠진 듯). 저자 코드를 고치지 않으려고 빈 모듈로 채운다.
"""
import os
import runpy
import sys
import types

for name in ('robot_model.acrobot',
             'robot_model.acrobot.acrobot_model',
             'robot_model.acrobot.observation_preprocesor',
             'robot_model.acrobot.acrobot_params'):
    sys.modules[name] = types.ModuleType(name)

runpy.run_path(os.path.join(os.environ['HPM_CODE'], 'main.py'), run_name='__main__')
