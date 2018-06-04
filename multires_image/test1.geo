image_path: "tiles"
image_width: 1000
image_height: 1000
tile_size: 1000

datum: "wgs84"
projection: "utm"

tiepoints:


- point:  [300,270    ,408495.3248920463, 5796155.580282022]
- point:  [150,612    ,408477.62267375714, 5796115.963393918]
- point:  [800,690    ,408551.3623101322, 5796105.240596578]




#to compute tiepoints for ros-morse simulation
#1. spawn drone at tiepoint -> retreive gps (lat,lon)
#2. get pixel coordinates of location where drone spawned (f.E.: gimp)
#3. compute utm with compute_utm script( uses utm library)
#4. tiepoints is then `-points:[pixel_x,pixel_y,utm_easting,utm_northing]`




#used to compute points:
# top-middle of uppermost car in top view
#latitude: 52.3081048906
#longitude: 13.6578301181

#middle of left side building left-bottom
#latitude: 52.3077458632
#longitude: 13.6575813155

#bottom right corner of bottom right building
#latitude: 52.3076617716
#longitude: 13.6586654947

# top-middle of uppermost car in top view
#copter2 = create_quadrotor(name='uav2')
#copter2.translate(x=-3.0, y=19.0, z=0.075)

#middle of left side building left-bottom
#copter3 = create_quadrotor(name='uav3')
#copter3.translate(x=-20.0, y=-21.0, z=0.075)

#bottom right corner of bottom right building
#copter4 = create_quadrotor(name='uav4')
#copter4.translate(x=54, y=-30.0, z=0.075)



