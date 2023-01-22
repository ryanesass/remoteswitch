# remoteswitch
Arduino Software for ESP8266 to interface with mosquito MQTT server. Written 2023
Remote computer switch
  This code runs a ESP8266 that handles switching on and off a computer. It is set to stay
  awake for only a minute unless it receives commands or is commanded to stay awake. 
  It can turn on and off a computer, as well as perform basic checks on the computer status
  and update the user about the boar operation and connection detals. It writes data on an 
  as needed basis to the RTC memory bank depending on any situations that have changed. 

  It connects to an MQTT server and only handles QoS 1 data packages when connected. It will
  notify the MQTT client when it connects, and when it is soon to be heading to sleep. It 
  will always send feedback commands when it receives commands, including unknown commands. 

  This is an ongoing project and will be updated as need be. 
  
  V1.0 - initial run, works and tested to full functionality
  V1.1 - updated the display to show the computer status when connecting, as well as did 
  some code cleanup. Also updated some display output changes to the countdown code to 
  display minutes instead of seconds only. 
  V1.2 - updated to sanatizePayload message commands, handle blank commands, provide status reports
  on RTC memory flags, user can now toggle publishing UTC or local time, and now user can 
  set specified awake times (in min), as well as specified hours for sleeping. For specified 
  sleeping, user must CONFIRM the sleep by calling sleepy5, and board will then sleep for
  input hours. Any other command (except countdown) will cancel out the order. There is a 
  maximum limit to sleeping. 
  V1.3 - added persistence messaging. Now the board will clear an incoming persistent message
  during initial startup. This is assuming it is receiving this command for itself. It also 
  clears it's own last persistent message it publishes to (usually the last sleep message). 
  Finally, it will publish a persistent message about the status of the computer. 
  V1.4 - revamped how handling the status recording of the computer is handled. it is now more stable
  and accurate. Also made changes to persistent sleep command - board will keep sleeping regardless 
  of resets. Now, it will also wait for computer status changes before going to sleep (both forced 
  and timeout) though the forced sleep can be overridden to sleep by commanding 2x when waiting. 
  V1.5 - fixed a bug with caffen8 command, updated RTC to updated correctly, and added software
  restart and hardware restart commands. Also, doubled the amount of values in state (to 20) to 
  handle the correction occurring during restart
  V1.6 - updated a few minor print typos and sequencing of printed information. Also, updated entire TZ
  list to include a sample from all 54 active time zones (as of Jan 2023). Finally, added a secondary
  TZ list to print out the TZ offset from UTC. A star (*) next to the offset indicates that TZ observes
  DST as well. 
  V1.7 - added hibernate function and changed travel sleep to have a 31 day window. Travel sleep and 
  hibernation both sleep for an extended time period, with travel sleep ending at a given time. Hibernate
  resumes long sleeping indefinitely. Both can be ended by caffenating or the Awake Commands. Also, minor 
  changes to memory handling. Number of output changes. 
  V1.8 - pulling out bugs in code, fixing the handling between the different functions and cleaning up 
  the leftover commented out code no longer needed. Also, downsized the handling of numbers being sent
  via MQTT to be a for loop instead of a nested if statement
  V1.9 - added commands to update all sleep length durations as well as printing out sleep durations. Also
  added last will and testiment call for connection as well as doing a graceful disconnect. Finally, added
  a channel and function to show when the device is expected to reconnect to the server. 
  V1.10 - fixed bug in night time sleep where it was setting to day time sleep, and another bug in 
  checkStayAsleep() where it tries to publish when it hasn't connected.

  Last Update: 16 Jan 2023 11:49PM PST -- Ryan Sass
  */
