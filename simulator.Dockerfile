FROM autodriveecosystem/autodrive_f1tenth_sim:2024-cdc-compete

RUN rm /usr/lib/x86_64-linux-gnu/*nvidia* && rm /usr/lib/x86_64-linux-gnu/*cuda*

COPY simulator-startup.bash simulator-startup.bash

ENTRYPOINT ["/bin/bash", "simulator-startup.bash"]