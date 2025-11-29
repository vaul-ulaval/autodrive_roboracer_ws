FROM autodriveecosystem/autodrive_roboracer_sim:2025-icra-practice

RUN rm /usr/lib/x86_64-linux-gnu/*nvidia* && rm /usr/lib/x86_64-linux-gnu/*cuda*

COPY simulator-startup.bash simulator-startup.bash

ENTRYPOINT ["/bin/bash", "simulator-startup.bash"]