# PHRI

workspaces for PHRI

## Developing, venvs and bulding
Newer Linux/Python installations don't like installing python packages system wide. 
Sometimes there are apt repos to install system wide, but only for the large, widely-used packages. 
So we do have to run with a venv. 

Create one with `python3 -m venv ~/venv/ros`.


For convenience i recommend setting these aliases in your `~/.bashrc`, then run it with `. ~/.bashrc`. 
```
alias sourceros='source /opt/ros/jazzy/setup.bash'
alias sourcelocal='source install/local_setup.bash'
alias sourcevenv='source ~/venv/ros/bin/activate'
alias exportvenv='export PYTHONPATH=~/venv/ros/lib/python3.12/site-packages:$PYTHONPATH'
alias sourceall='cd [PATH_TO_WS] && sourceros && sourcelocal && sourcevenv && exportvenv'
alias buildros='colcon build --symlink-install' 
```

Instead of `colcon build`, i recommend using `colcon build --symlink-install`, which installs symlinks to the actual python files, so we dont need to recompile, whenever we change python files. Only when we need add files. Great for development!

### Installation requirements
Finally, install all the following requirements:

For Audio (needs native Linux, didn't work with wsl for me)
```
sudo apt install portaudio19-dev espeak-ng alsa-utils
```

Inside a venv:
```
pip install "setuptools==68.1.2" "empy==3.3.4" lark catkin_pkg flask vosk bleak piper-tts sounddevice soundfile faster-whisper sentence-transformers
```

To download models:
```
cd ros_ws/src/brewbot/brewbot/models
```

SentenceTransformers: 
```
python3 -c "from sentence_transformers import SentenceTransformer; SentenceTransformer('all-MiniLM-L6-v2').save('all-MiniLM-L6-v2'); SentenceTransformer('all-mpnet-base-v2').save('all-mpnet-base-v2')"
```

Vosk:
```
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
rm vosk-model-small-en-us-0.15.zip
```

Piper (TTS):
```
mkdir ~/piper
cd ~/piper
wget https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_GB/alan/medium/en_GB-alan-medium.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_GB/alan/medium/en_GB-alan-medium.onnx.json
```

A few notes: 
- setuptool needs to be downgraded, as newer version don't support Colcons --editable anymore. 
- sentence transformers is by far the largest pip module. Don't install if you are not going to run it.