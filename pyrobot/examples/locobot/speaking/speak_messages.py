# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Example for commanding robot with the src without using planner
"""

import time

from pyrobot.locobot.speaker import Speaker

def main():
    message = "Hello world, OKAY"
    locobot_speaker = Speaker()
    locobot_speaker.speak(message)

if __name__ == "__main__":
    main()
