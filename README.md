# PolygonTriangulation_EarClipping2D

The polygon triangulation is one critical algorithm for dealing with most famous problem in computational geoemtery "The art-gallery problem". 

Apart from the naive, monotone partioning and convex partioning algorithms, the ear clipping algorithm is also one crucial implementation for dealing with the art-gallery problem. 

This algorithm triangulates a polygon using earclipping algorithm. This functions at a complexity of O(n^2). It takes the polygon vertices as a input from a text file.

Demo Output 1:- 

![Triangulation](https://user-images.githubusercontent.com/29711990/167002784-c97deb3c-dac6-4fc2-9644-d37add8cb560.png)

Demo Output 2:- 

![Figure_3](https://user-images.githubusercontent.com/29711990/167003675-caed7ce4-8d50-4c90-aa82-06afc8d718c0.png)

Demo Output 3:- 

![Figure_2](https://user-images.githubusercontent.com/29711990/167003680-92a54e25-b64c-43b4-b1de-914be36e5828.png)

The above code does not use any pre-built compuation geometry libraries to do the partioning, rather it is a raw implemention of the ear clipping algorithm.
