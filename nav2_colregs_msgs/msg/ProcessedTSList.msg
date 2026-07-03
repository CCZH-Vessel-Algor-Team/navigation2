# ProcessedTSList.msg
# Batch of processed target ships from TS State Manager.
# Published at 10Hz on /processed_ts_list.

std_msgs/Header header
ProcessedTS[] ships
