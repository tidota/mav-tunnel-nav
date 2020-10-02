# TODO list

## inter-robot communication

### Beacon
- beacon thread
  - at some interval
    - broadcast beacons
    - wait for some time
    - update the neighbors list

- callback (when receiving a beacon packet)
  - reply back

### Data exchange

- thread (maybe the main process of rbpf? or a separate thread?)

  - at some probability
    - randomly choose a receiver from the neighbors list
    - send a request to exchange
    - busy-wait for a reply and time
    - if no timed out and acc received
      - lock the data
      - calc weights for omega and 1-omega
      - send particles with weights of 1-omega
      - wait for particles w/ weights from the other
      - resampling
      - unlock the data

- callback (receiving a request to exchange)
  - if the data is not locked already
    - lock the data
    - send acc
    - calc weights for omega and 1-omega
    - send particles with weights of 1-omega
    - wait for particles w/ weights from the other
    - resampling
  - else
    - send rejection

- callback (receiving data)
  - enque?

- callback (receiving acc)
  - raise a flag?
