# TODOs
- [x] Add camera sensor and try them in sync mode
- [x] Add lidar sensor and try them in sync mode
- [x] Add Semantic Segmentation sensor and try them in sync mode
- [ ] Weather conditions (doesn't work with 0.10.0 ?)
  - https://carla.org/2024/12/19/release-0.10.0/ 
  - Weather is fixed to daylight setting. Clouds, rain, fog and sun position cannot be modified
  - Maps: CARLA Towns 1-9, 11, 12, 13 and 15 have not been upgraded and are not included, Town 10 has been upgraded. Many of the assets from older towns are still available in the content library.
- [x] Understand how to spawn something in front of the ego vehicle
  - maybe waypoints
- [x] Find static anomalies assets 
  - [x] Do different semantic tabs for the anomalies
- [x] Make the simulation automatic (spawn a specific point)
  - check the spawn points in the map if they are valid
- [x] LiDAR
  - vedere configurazione!


- [x] Ereditare tutte le classi da anoamly
- [x] Semantic Lidar
- [ ] Radar
- [x] Depth camera
- [ ] Fare durata massima (5 sec) / supera l'anomalia. Stoppare se tocca anomalia
- [x] Aumentare frame di cattura
- [x] Provare instance anomaly (instance camera)
- [ ] Check LiDAR under different weather conditions
- [ ] Documentation
- [ ] Packages