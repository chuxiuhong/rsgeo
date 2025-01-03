[![Build Status](https://travis-ci.com/chuxiuhong/rsgeo.svg?branch=main)](https://travis-ci.com/chuxiuhong/rsgeo)
[![crates.io](https://img.shields.io/crates/v/rsgeo)](https://crates.io/crates/rsgeo)

rsgeo 是一个用Rust开发的地理计算库

## 安装

在`Cargo.toml`里加上

```rust
[dependencies]
rsgeo = "0.1.4"
```

## 使用

用纬度和经度来创建两个点(`Point`)，并且计算两点间的距离（地球大圆距离）

```rust
use rsgeo::prelude::*;
let p1 = Point::new(30.12, 125.26).unwrap(); 
let p2 = Point::new(32.54, 107.15).unwrap();
println!("{}",p1.distance(&p2)); //1740784.4 
```

计算从`p1`到`p2`的与北方之间的夹角

```rust
println!("{}",p1.degree(&p2)); //-82.38877
```

创建一个位置(`Location`)

```rust
let p = Point::new(35.12,46.15).unwrap();
let loc = Location::new(p,232500);
// Location相当于一个Point加一个Unix时间戳
```

计算`Location`和`Point`之间的距离

```rust
println!("{}",loc.distance_from_point(&p1)); //7237628.5
```

计算两个位置之间的速度

```rust
let loc1 = Location::new(Point::new(25.12,110.15).unwrap(),100);
let loc2 = Location::new(Point::new(25.119,109.995).unwrap(),3700);
assert!(loc1.speed(&loc2) - 4.3396673 < 1e-6); // m/s
```

创建区域（支持矩形和圆形）并测试点是否在区域内部

```rust
let ra = RecArea::new(35.0, 36.0, 110.0, 115.0).unwrap();
let pa = Point::new(32.0,112.0).unwrap();
let pb = Point::new(35.1,112.0).unwrap();
assert!(!ra.contains(&pa));
assert!(ra.contains(&pb));
let ca = CircleArea::new(Point::new(35.0, 110.0).unwrap(), 350_000.0);
let pa = Point::new(32.0, 112.0).unwrap();
let pb = Point::new(35.1, 112.0).unwrap();
assert!(!ca.contains(&pa));
assert!(ca.contains(&pb));
```

创建多边形（凹凸多边形均可）并测试点是否在多边形内部

```rust
let pa = Point::new(32.0,112.0).unwrap();
let pb = Point::new(35.1,112.0).unwrap();
let pc = Point::new(35.3,112.5).unwrap();
let pg = Polygon::new(vec![pa,pb,pc]);
assert!(pg.is_err());
// polygon must have more than 3 points.The first point must be the same with the last point.
// the polygon can be convex or concave whatever
let pg = Polygon::new(vec![pa,pb,pc,pa]);
assert_eq!(pg.unwrap().len(),4);
let pt1 =  Point::new(35.0,112.2).unwrap();
let pt2 =  Point::new(34.29,110.3).unwrap();
assert!(pg.contains(&pt1));
assert!(!pg.contains(&pt2));
```

创建一个轨迹`Trajectory`，轨迹相当于一个有序的位置`Location`的集合。要求后面的位置在时间上必须晚于前面的位置。

```rust
// A trajectory is a vector of some locations which must be chronological order.
let mut t = Trajectory::new();
let mut t = Trajectory::with_capacity(); // like a vector
let loc1 = Location::new(Point::new(25.11,120.98).unwrap(),0);
let loc2 = Location::new(Point::new(26.2,121.1).unwrap(),7200);
let loc3 = Location::new(Point::new(26.3,121.3).unwrap(),14400);
t.push_location(&loc1);
t.push_location(&loc2);
t.push_location(&loc3);
let mut t = Trajectory::from(vec![loc1,loc2,loc3].as_slice()).unwrap(); // or initialize with a slice of locations
```

获取轨迹内位置点的总数量

```rust
assert_eq!(t.len(),3);
```

测试轨迹是否经过区域或者多边形

```
let area1 = RecArea::new(25.07,27.12,119.9,125.0).unwrap();
assert!(t.pass_rec_area(&area1));
```

获取轨迹的平均速度和最大速度

```rust
let loc1 = Location::new(Point::new(25.11,120.98).unwrap(),0);
let loc2 = Location::new(Point::new(26.2,121.1).unwrap(),7200);
let loc3 = Location::new(Point::new(26.3,121.3).unwrap(),14400);
let mut t = Trajectory::with_capacity(3);
t.push_location(&loc1);
t.push_location(&loc2);
t.push_location(&loc3);
assert!((t.max_speed().unwrap() - 16.9352).abs() < 1e-3);
assert!((t.mean_speed().unwrap() - 10.055225).abs() < 1e-6);
```

获取轨迹的总长度

```rust
assert!((t.sum_distance() - 144795.25).abs() < 1e6);
```

比较两个轨迹的相似度，采用STLC算法，该算法在我前面的文章已经介绍过。

```rust
use rsgeo::prelude::*;
use rsgeo::measure::stlc_trajectory_similarity;
assert!((stlc_trajectory_similarity(&t, &t, 0.5).unwrap()-1.0).abs() < 1e-6);
// stlc is spatiotemporal linear combin distance
// the function returns similarity which is in (0,1]
```

计算轨迹的转向角度序列

```rust
let loc1 = Location::new(Point::new(0.0, 0.0).unwrap(), 0);
let loc2 = Location::new(Point::new(0.0, 1.0).unwrap(), 1);
let loc3 = Location::new(Point::new(1.0, 1.0).unwrap(), 2);
let loc4 = Location::new(Point::new(1.0, 0.0).unwrap(), 3);
let mut t = Trajectory::with_capacity(4);
t.push_location(&loc1).unwrap();
t.push_location(&loc2).unwrap();
t.push_location(&loc3).unwrap();
t.push_location(&loc4).unwrap();
println!("{:?}", t.turn_angles().unwrap()); // [90.0, -90.0]
```

计算轨迹的总转向角度和平均转向角度

```rust
println!("{}", t.total_turn_angle().unwrap()); // 180.0
println!("{}", t.average_turn_angle().unwrap()); // 90.0
```

计算多边形的面积（平方米）

```rust
let pa = Point::new(32.0,112.0).unwrap();
let pb = Point::new(35.1,112.0).unwrap();
let pc = Point::new(35.3,112.5).unwrap();
let pg = Polygon::new(vec![pa,pb,pc,pa]).unwrap();
println!("{}", pg.area()); // 返回多边形的面积，单位为平方米
```
