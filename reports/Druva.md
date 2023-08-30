
### ERC Hackathon - My Experience ( Druva)


## General Thoughts
 - Initially we had a lot of enthusiasm for the hackathon and had a couple of
   group discussions where all of us were contributing ideas, we hadn’t read
   the full description yet and we were having several wild ideas, now that i
   think about it, we wouldn’t have even been able to start implementing those
   ideas. 
 - After we got the full problem description, it was clear what we had to do,
   we were already divided into three teams of two for each of the three
   subsystems. I was assigned automation along with Param Gandhi. We discussed
   about our tasks and the feasibility of completing it, we even tried to setup
   timelines for the tasks so we could progress properly. We ended up setting a
   deadline for finishing our learning part/ setting up workspace and to start
   working, we also wanted to have a proper meet after that. 
 - This deadline was a week into the holidays, I wasn't able to set up my
   workspace as I needed a ram upgrade to run ROS, I ended up doing no work
   till august. We decided that I was going to do the path planning and colour
   detection and Param was going to do the pid controller, I had a bit of
   technical experience with both path planning and colour detection so the
   plan was to finish it up quickly and start learning about the controller. 
 - By August I finally got my RAM upgrade and got to working on the path
   planning algorithm, I used some of my old code from my path planning mini
   project and managed to get some results. However, I wasn't able to do much
   with ROS and I realised i needed to learn a lot more about ROS. 
 - After coming back to campus, I slowly started working again and integrated
   the path planner with ROS, completed colour detection and waited for param
   to finish the controller. 
 - Once he was done with the controller I integrated the algorithm with the
   path planner and started testing it, after tweaking the variables in the
   algorithms and testing it several times, it was able to properly reorient
   itself after getting stuck or going off path.

## Technical Overview ### path planning 
  - Talked with Param and decided that we were going to generate a grid array
    from the walls and generate a node graph from that and feed it to djikstra. 
  - Generating a grid array was simple enough, split the canvas into a n x n
    grid and traverse through each region and check if there is a wall in the
    region. make a n x n array and flag walled regions with 1 and paths with 0. 
  - For generating a node graph, My old algorithm worked with mazes with only a
    single point width ( the pathways were one unit wide) , the new mazes had
    pathways which where several units wide, so I had to tweak the node
    generating algorithm to generate nodes only when the width of the pathway
    changes or if there is a dead end. This made sure that all the important
    nodes were generated for the graph
  - Initially we weren't sure how we were going to determine the shortest path
    from start to several target nodes, we decided to find the closest node
    using the distance formula and planning a path to that node using djikstra. 
  - I already had djikstra implemented for the old mazes, with a few tweaks it
    was working for the new wider maze. 
  - The one drawback on using this method to find the path is that traversing
    through n x n grid and generating nodes has a bad time complexity, choosing
    n is also a problem, choose too less and you miss out on paths and the maze
    remains unsolved, choose too big and it takes too long to find a path. I
    chose to loop through values of n from 50 to 100, till a path is found. 
  - After the algorithm was completed, I had to integrate this with ROS,
    basically had to use the Point datatype from the geometry_msgs library to
    publish this data to the topic. ### colour detection 
  - Colour detection was relatively straightforward to implement, I had to
    recognise colours from the image and publish tasks based on that. The
    trickier part was differentiating between the several cones, the node only
    gets subscribed to the raw camera feed, so it couldn't realistically
    differentiate between different images of the same cone and different
    cones.
  - I used a mask with the red and blue filters, and when the average pixel
    value crossed a threshold i set a state variable to extracting and
    published the extract based on the average pixel value. 
  - Assumed extraction would take n seconds and didn't update the variable for
    those n seconds, after that it again resumed to the initiaal state and
    would publish status depending on the average pixel value after the
    masking. 
  - once the number of publications reached the total value, published the
    finished extraction message.

### Controller 
 - Param had implemented the algorithm for the pid controller, it could
   basically adjust its angular velocity to face a point, and then move till it
   reaches it. 
 - This algorithm couldn't handle obstacles or even slight deviation from the
   paths due to small errors.
 - I tweaked the algorithm by adding a check for being stuck and making it
   reorient itself, but the implementation is very crude as it just checks the
   position of the bot 5 seconds ago and compares it with the current position
   to determine if its stuck. 
 - I also re-implemented the angular and linear velocity shifts with recursion
   so that the bot can self correct its course multiple times before reaching
   the goal. Initially it would only face the point and move. 
 - Tweaked the goal distance parameter ( distance around goal in which it
   should consider it as reached the goal) and the absolute angle parameter,
   which allows to self correct the path. 
 - Also integrated the algorithm with the goal points from the path planner
   node. 
