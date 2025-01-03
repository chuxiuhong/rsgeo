#![allow(unused)]

use std::collections::{HashMap, HashSet};
use std::hash::{Hash, Hasher};

///
/// Point data type
///
/// Including `latitude` and `longitude` as f32
///
#[derive(Debug, Copy, Clone)]
pub struct Point {
    pub(crate) lat: f32,
    pub(crate) long: f32,
}

impl PartialEq for Point {
    fn eq(&self, other: &Self) -> bool {
        (self.lat - other.lat).abs() < 1e-6 && (self.long - other.long).abs() < 1e-6
    }
}

impl Hash for Point {
    fn hash<H: Hasher>(&self, state: &mut H) {
        let lat_bits = self.lat.to_bits();
        let long_bits = self.long.to_bits();
        lat_bits.hash(state);
        long_bits.hash(state);
    }
}

impl Eq for Point {}

#[inline]
fn valid_lat(lat: f32) -> bool {
    (-90.0..=90.0).contains(&lat)
}

#[inline]
fn valid_long(long: f32) -> bool {
    (-180.0..=180.0).contains(&long)
}

///
/// transform a degree to radians value
///
/// # Example
/// ```
/// use rsgeo::prelude::radians;
/// use std::f32::consts::PI;
/// let a = 180.0f32;
/// let b = 90.0f32;
/// let c = 30.0f32;
/// let d = -5.0f32;
/// assert!((radians(a) - PI).abs() < 1e-6);
/// assert!((radians(b) - (PI / 2.0)).abs() < 1e-6);
/// assert!((radians(c) - (PI / 6.0)).abs() < 1e-6);
/// assert!((radians(d) - -0.0872664).abs() < 1e-6);
/// ```
#[inline]
pub fn radians(deg: f32) -> f32 {
    (deg / 180.0) * std::f32::consts::PI
}

///
/// transform a radians to degree value
///
/// # Example
/// ```
/// use rsgeo::prelude::degree;
/// use std::f64::consts::PI;
/// let a = (PI / 2.0) as f32;
/// let b = (PI / 4.0) as f32;
/// assert!((degree(a) - 90.0).abs() < 1e-6);
/// assert!((degree(b) - 45.0).abs() < 1e-6);
/// ```
pub fn degree(rad: f32) -> f32 {
    (rad * 180.0) / (std::f64::consts::PI as f32)
}

impl Point {
    ///
    /// `latitude` should be smaller than 90.0 and greater than -90.0
    ///
    /// `longitude` should be smaller than 180.0 and greater than -180.0
    ///
    /// # Example
    /// ```
    /// use rsgeo::prelude::Point;
    /// let p1 = Point::new(30.12, 125.26).unwrap();
    /// let p2 = Point::new(32.54, 107.15).unwrap();
    /// let p3 = Point::new(30.12, 125.26).unwrap();
    /// assert_ne!(p1, p2);
    /// assert_eq!(p1, p3);
    /// ```
    pub fn new(lat: f32, long: f32) -> Result<Self, &'static str> {
        if !valid_lat(lat) {
            Err("Invalid latitude!")
        } else if !valid_long(long) {
            Err("Invalid longitude!")
        } else {
            Ok(Self {
                lat,
                long,
            })
        }
    }

    ///
    /// calculate distance between two points in earth sphere mode
    ///
    /// # Example
    ///```
    /// use rsgeo::prelude::Point;
    /// let p1 = Point::new(0.0, 0.0).unwrap();
    /// let p2 = Point::new(0.0, 0.0).unwrap();
    /// let p3 = Point::new(28.478, 179.395).unwrap();
    /// let p4 = Point::new(28.33, -177.547).unwrap();
    /// assert!(p1.distance(&p2).abs() < 0.001);
    /// assert!(p3.distance(&p4).abs() - 299878.88958 < 0.1);
    /// ```
    #[inline]
    pub fn distance(&self, other: &Self) -> f32 {
        let (mut lat1, mut long1, mut lat2, mut long2) = (self.lat, self.long, other.lat, other.long);
        if long1 < 0.0 {
            long1 += 360.0;
        }
        if long2 < 0.0 {
            long2 += 360.0;
        }
        let (lat1, long1, lat2, long2) = (radians(lat1), radians(long1), radians(lat2), radians(long2));
        let dlong = long2 - long1;
        let dlat = lat2 - lat1;
        let a = (dlat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (dlong / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().asin();
        let i = 6378135f32;
        (c * i).abs()
    }

    ///
    /// calculate distance between point and location
    ///
    #[inline]
    pub fn distance_from_location(&self, other: &Location) -> f32 {
        self.distance(&other.pos)
    }


    ///
    /// calculate the shortest distance between point and trajectory
    ///
    /// the time complexity is `O(n)`,which `n` is trajectory.len()
    ///
    /// # Example
    /// ```
    /// use rsgeo::prelude::{Trajectory,Location,Point};
    /// let p = Point::new(26.301,121.302).unwrap();
    /// let loc1 = Location::new(Point::new(25.11,120.98).unwrap(),100);
    /// let loc2 = Location::new(Point::new(26.2,121.1).unwrap(),150);
    /// let loc3 = Location::new(Point::new(26.3,121.3).unwrap(),200);
    /// let mut t = Trajectory::with_capacity(3);
    /// t.push_location(&loc1);
    /// t.push_location(&loc2);
    /// t.push_location(&loc3);
    /// assert!((p.shortest_distance_from_trajectory(&t).unwrap() - p.distance_from_location(&loc3)).abs() < 1e-6);
    /// ```
    pub fn shortest_distance_from_trajectory(&self, other: &Trajectory) -> Option<f32> {
        if other.is_empty() { return None; }
        let mut min_distance = 1e10;
        for location in &other.locations {
            let dist = self.distance(&location.pos);
            if dist < min_distance {
                min_distance = dist;
            }
        }
        Some(min_distance)
    }


    /// Calculate the degree which between the line and the north
    ///
    /// degree in [-90.0,+90.0]
    ///
    /// # Example
    /// ```
    /// use rsgeo::prelude::*;
    /// let p1 = Point::new(10.0,150.0).unwrap();
    /// let p2 = Point::new(20.0,150.0).unwrap();
    /// let p3 = Point::new(10.0,160.0).unwrap();
    ///
    /// assert!((p1.degree(&p2)).abs() < 1e-5);
    /// assert!((p1.degree(&p3) - 90.0).abs() < 1e-5);
    /// ```
    pub fn degree(&self, other: &Point) -> f32 {
        degree(((other.long - self.long) / (other.lat - self.lat + 1e-6)).atan())
    }
}


///
/// Location is a Point with unix timestamp
///
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub struct Location {
    pub(crate) pos: Point,
    pub(crate) timestamp: u64,
}

impl Location {
    /// # Example
    /// ```
    /// use rsgeo::prelude::{Point,Location};
    /// let p = Point::new(35.12,46.15).unwrap();
    /// let loc = Location::new(p,123);
    /// ```
    pub fn new(pos: Point, timestamp: u64) -> Self {
        Self {
            pos,
            timestamp,
        }
    }

    ///
    /// calculate distance between location and location
    ///
    pub fn distance(&self, other: &Self) -> f32 {
        self.pos.distance(&other.pos)
    }

    ///
    /// calculate distance between location and Point
    ///
    pub fn distance_from_point(&self, other: &Point) -> f32 {
        self.pos.distance(&other)
    }

    /// calculate distance between location and Trajectory
    pub fn distance_from_trajectory(&self, other: &Trajectory) -> Option<f32> {
        self.pos.shortest_distance_from_trajectory(&other)
    }

    ///
    /// calculate speed between two locations,return m/s
    ///
    /// # Example
    /// ```
    /// use rsgeo::prelude::{Point,Location};
    /// let loc1 = Location::new(Point::new(25.12,110.15).unwrap(),100);
    /// let loc2 = Location::new(Point::new(25.119,109.995).unwrap(),3700);
    /// assert!(loc1.speed(&loc2) - 4.3396673 < 1e-6);
    /// ```
    pub fn speed(&self, other: &Self) -> f32 {
        if self.timestamp > other.timestamp {
            self.distance(other) / ((self.timestamp - other.timestamp) as f32)
        } else {
            self.distance(other) / ((other.timestamp - self.timestamp) as f32)
        }
    }

    /// return the shortest time distance from trajectory
    ///
    /// # Example
    /// ```
    /// use rsgeo::prelude::*;
    /// let loc = Location::new(Point::new(25.12,110.15).unwrap(),100);
    /// let loc1 = Location::new(Point::new(25.11,120.98).unwrap(),0);
    /// let loc2 = Location::new(Point::new(26.2,121.1).unwrap(),7200);
    /// let loc3 = Location::new(Point::new(26.3,121.3).unwrap(),14400);
    /// let mut t = Trajectory::with_capacity(3);
    /// t.push_location(&loc1);
    /// t.push_location(&loc2);
    /// t.push_location(&loc3);
    /// assert_eq!(loc.time_shortest_distance_from_trajectory(&t).unwrap(),100);
    /// ```
    pub fn time_shortest_distance_from_trajectory(&self, other: &Trajectory) -> Option<u64> {
        if other.is_empty() { return None; }
        let mut min_time_distance = 1e10 as u64;
        for location in &other.locations {
            if location.timestamp >= self.timestamp && location.timestamp - self.timestamp < min_time_distance {
                min_time_distance = location.timestamp - self.timestamp;
            } else if location.timestamp <= self.timestamp && self.timestamp - location.timestamp < min_time_distance {
                min_time_distance = self.timestamp - location.timestamp;
            }
        }
        Some(min_time_distance)
    }

    pub fn degree(&self, other: &Location) -> f32 {
        self.pos.degree(&other.pos)
    }
}

/// a trajectory is a vector of location which is in time-order
#[derive(Debug)]
pub struct Trajectory {
    pub(crate) locations: Vec<Location>,
}

impl Default for Trajectory {
    fn default() -> Self {
        Self::new()
    }
}


impl Trajectory {
    ///
    /// return a Trajectory object with no locations
    ///
    pub fn new() -> Self {
        Self {
            locations: Vec::new(),
        }
    }

    ///
    /// return a Trajectory object with capacity
    ///
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            locations: Vec::with_capacity(capacity)
        }
    }

    pub fn from(locs: &[Location]) -> Result<Self, &'static str> {
        if !locs.is_sorted_by_key(|x| x.timestamp) { 
            return Err("Wrong time order of points!"); 
        }
        Ok(Self {
            locations: locs.to_vec()
        })
    }

    ///
    /// add new locations to `Trajectory`
    ///
    /// # Example
    /// ```
    /// use rsgeo::prelude::{Point,Location,Trajectory};
    /// let loc1 = Location::new(Point::new(25.11,120.98).unwrap(),100);
    /// let loc2 = Location::new(Point::new(26.2,121.1).unwrap(),150);
    /// let loc3 = Location::new(Point::new(26.3,121.3).unwrap(),200);
    /// let mut t = Trajectory::with_capacity(3);
    /// t.push_location(&loc1);
    /// t.push_location(&loc2);
    /// t.push_location(&loc3);
    /// assert_eq!(t.len(),3);
    /// assert!(t.push_location(&Location::new(Point::new(26.5,121.4).unwrap(),190)).is_err()); // Invalid time order
    /// ```
    pub fn push_location(&mut self, loc: &Location) -> Result<(), &'static str> {
        if let Some(x) = self.locations.last() {
            if x.timestamp >= loc.timestamp { return Err("New location timestamp is smaller than the trajectory's timestamp"); }
        }
        self.locations.push(*loc);
        Ok(())
    }

    /// Returns the number of Locations in Trajectory
    ///
    /// # Example
    /// ```
    /// use rsgeo::prelude::{Point,Location,Trajectory};
    /// let loc1 = Location::new(Point::new(25.11,120.98).unwrap(),100);
    /// let loc2 = Location::new(Point::new(26.2,121.1).unwrap(),150);
    /// let loc3 = Location::new(Point::new(26.3,121.3).unwrap(),200);
    /// let mut t = Trajectory::with_capacity(3);
    /// t.push_location(&loc1);
    /// t.push_location(&loc2);
    /// t.push_location(&loc3);
    /// assert_eq!(t.len(),3);
    /// ```
    pub fn len(&self) -> usize {
        self.locations.len()
    }

    /// judge if the trajectory passed rectangle area
    ///
    /// The time complexity is (n),which n is the length of trajectory
    /// # Example
    ///
    /// ```
    /// use rsgeo::prelude::{RecArea,Point,Trajectory,Location};
    /// let loc1 = Location::new(Point::new(25.11,120.98).unwrap(),100);
    /// let loc2 = Location::new(Point::new(26.2,121.1).unwrap(),150);
    /// let loc3 = Location::new(Point::new(26.3,121.3).unwrap(),200);
    /// let mut t = Trajectory::with_capacity(3);
    /// t.push_location(&loc1);
    /// t.push_location(&loc2);
    /// t.push_location(&loc3);
    /// let area1 = RecArea::new(35.07,35.12,111.9,112.0).unwrap();
    /// let area2 = RecArea::new(26.15,26.201,120.0,121.15).unwrap();
    /// assert!(!t.pass_rec_area(&area1));
    /// assert!(t.pass_rec_area(&area2));
    /// ```
    pub fn pass_rec_area(&self, area: &RecArea) -> bool {
        for location in &self.locations {
            if area.contains(&location.pos) { return true; }
        }
        false
    }

    pub fn pass_circle_area(&self, area: &CircleArea) -> bool {
        for location in &self.locations {
            if area.contains(&location.pos) { return true; }
        }
        false
    }

    pub fn pass_polygon(&self, poly: &Polygon) -> bool {
        for location in &self.locations {
            if poly.contains(&location.pos) { return true; }
        }
        false
    }
    /// Find the most fast speed of a trajectory
    ///
    /// Time complexity is O(n),which n is the length of trajectory
    ///
    /// # Example
    /// ```
    /// use rsgeo::prelude::{Point,Location,Trajectory};
    /// let loc1 = Location::new(Point::new(25.11,120.98).unwrap(),0);
    /// let loc2 = Location::new(Point::new(26.2,121.1).unwrap(),7200);
    /// let loc3 = Location::new(Point::new(26.3,121.3).unwrap(),14400);
    /// let mut t = Trajectory::with_capacity(3);
    /// t.push_location(&loc1);
    /// t.push_location(&loc2);
    /// t.push_location(&loc3);
    /// assert!((t.max_speed().unwrap() - 16.9352).abs() < 1e-3);
    /// ```
    pub fn max_speed(&self) -> Option<f32> {
        if self.len() < 2 { return None; }
        let locations = self.locations.as_slice();
        let mut max_speed = 0.0f32;
        for i in 0..locations.len() - 1 {
            let t_speed = locations[i].speed(&locations[i + 1]);
            if t_speed > max_speed {
                max_speed = t_speed;
            }
        }
        Some(max_speed)
    }

    /// Calculate the mean speed of trajectory.The trajectory
    ///
    /// The time complexity is O(n),which n is the length of trajectory.
    /// # Example
    ///
    /// ```
    /// use rsgeo::prelude::{Point,Location,Trajectory};
    /// let loc1 = Location::new(Point::new(25.11,120.98).unwrap(),0);
    /// let loc2 = Location::new(Point::new(26.2,121.1).unwrap(),7200);
    /// let loc3 = Location::new(Point::new(26.3,121.3).unwrap(),14400);
    /// let mut t = Trajectory::with_capacity(3);
    /// t.push_location(&loc1);
    /// t.push_location(&loc2);
    /// t.push_location(&loc3);
    /// assert!((t.mean_speed().unwrap() - 10.055225).abs() < 1e-6);
    /// ```
    pub fn mean_speed(&self) -> Option<f32> {
        if self.len() < 2 { return None; }
        let locations = self.locations.as_slice();
        let mut path_length = 0f32;
        let mut path_time = 0u64;
        for i in 0..locations.len() - 1 {
            path_length += locations[i + 1].pos.distance(&locations[i].pos);
            path_time += locations[i + 1].timestamp - locations[i].timestamp;
        }
        Some(path_length / (path_time as f32))
    }

    /// return the sum of degree from a trajectory
    ///
    /// # Example
    ///
    /// ```
    /// use rsgeo::prelude::*;
    /// let loc1 = Location::new(Point::new(25.11,120.98).unwrap(),0);
    /// let loc2 = Location::new(Point::new(25.11,121.1).unwrap(),7200);
    /// let loc3 = Location::new(Point::new(25.11,121.3).unwrap(),14400);
    /// let mut t = Trajectory::with_capacity(3);
    /// t.push_location(&loc1);
    /// t.push_location(&loc2);
    /// t.push_location(&loc3);
    /// assert!(t.degrees().unwrap().abs() < 1e-3);
    /// ```
    pub fn degrees(&self) -> Option<f32> {
        if self.len() < 3 { return None; }
        let mut all_degree = 0f32;
        let mut last_degree = self.locations[0].pos.degree(&self.locations[1].pos);
        for i in 1..self.len() - 1 {
            all_degree += (last_degree - self.locations[i].pos.degree(&self.locations[i + 1].pos)).abs();
            last_degree = self.locations[i].pos.degree(&self.locations[i + 1].pos);
        }
        Some(all_degree / ((self.len() - 1) as f32))
    }
    /// Returns true if the trajectory contains no elements.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn sum_distance(&self) -> f32 {
        self.locations.windows(2)
            .map(|pair| pair[0].distance(&pair[1]))
            .sum()
    }
    #[inline]
    pub fn sum_time(&self) -> u64 {
        if self.locations.len() < 2 { 0 } else { self.locations.last().unwrap().timestamp - self.locations.first().unwrap().timestamp }
    }


    pub fn turn_angles(&self) -> Option<Vec<f32>> {
        if self.len() < 3 {
            return None;
        }
        
        let mut angles = Vec::with_capacity(self.len() - 2);
        
        for window in self.locations.windows(3) {
            let p1 = &window[0].pos;
            let p2 = &window[1].pos;
            let p3 = &window[2].pos;
            
            // 计算向量p1->p2和p2->p3
            let v1_lat = p2.lat - p1.lat;
            let v1_long = p2.long - p1.long;
            let v2_lat = p3.lat - p2.lat;
            let v2_long = p3.long - p2.long;
            
            // 计算两个向量的点积
            let dot_product = v1_lat * v2_lat + v1_long * v2_long;
            
            // 计算两个向量的叉积的z分量
            let cross_product = v1_lat * v2_long - v1_long * v2_lat;
            
            // 计算向量的模
            let v1_length = (v1_lat * v1_lat + v1_long * v1_long).sqrt();
            let v2_length = (v2_lat * v2_lat + v2_long * v2_long).sqrt();
            
            // 计算夹角（弧度）
            let angle = (dot_product / (v1_length * v2_length)).acos();
            
            // 根据叉积的符号确定转向方向
            let signed_angle = if cross_product >= 0.0 {
                angle
            } else {
                -angle
            };
            
            // 转换为角度并添加到结果中
            angles.push(degree(signed_angle));
        }
        
        Some(angles)
    }

    /// 计算轨迹的总转向角（单位：度）
    /// 
    /// 总转向角是指轨迹中所有转向角的绝对值之和。
    /// 这个值可以用来衡量轨迹的总体曲折程度。
    /// 
    /// 如果轨迹点数少于3个，返回None。
    /// 
    /// # Example
    /// ```
    /// use rsgeo::prelude::*;
    /// let loc1 = Location::new(Point::new(0.0, 0.0).unwrap(), 0);
    /// let loc2 = Location::new(Point::new(0.0, 1.0).unwrap(), 1);
    /// let loc3 = Location::new(Point::new(1.0, 1.0).unwrap(), 2);
    /// let loc4 = Location::new(Point::new(1.0, 0.0).unwrap(), 3);
    /// let mut t = Trajectory::with_capacity(4);
    /// t.push_location(&loc1).unwrap();
    /// t.push_location(&loc2).unwrap();
    /// t.push_location(&loc3).unwrap();
    /// t.push_location(&loc4).unwrap();
    /// let total = t.total_turn_angle().unwrap();
    /// assert!((total - 180.0).abs() < 1e-6); // 总共转了180度(90度 + 90度)
    /// ```
    pub fn total_turn_angle(&self) -> Option<f32> {
        self.turn_angles().map(|angles| {
            angles.iter().map(|angle| angle.abs()).sum()
        })
    }

    /// 计算轨迹的平均转向角（单位：度）
    /// 
    /// 平均转向角是总转向角除以转向次数。
    /// 这个值可以用来衡量轨迹的平均曲折程度。
    /// 
    /// 如果轨迹点数少于3个，返回None。
    /// 
    /// # Example
    /// ```
    /// use rsgeo::prelude::*;
    /// let loc1 = Location::new(Point::new(0.0, 0.0).unwrap(), 0);
    /// let loc2 = Location::new(Point::new(0.0, 1.0).unwrap(), 1);
    /// let loc3 = Location::new(Point::new(1.0, 1.0).unwrap(), 2);
    /// let loc4 = Location::new(Point::new(1.0, 0.0).unwrap(), 3);
    /// let mut t = Trajectory::with_capacity(4);
    /// t.push_location(&loc1).unwrap();
    /// t.push_location(&loc2).unwrap();
    /// t.push_location(&loc3).unwrap();
    /// t.push_location(&loc4).unwrap();
    /// let avg = t.average_turn_angle().unwrap();
    /// assert!((avg - 90.0).abs() < 1e-6); // 平均每次转90度
    /// ```
    pub fn average_turn_angle(&self) -> Option<f32> {
        if self.len() < 3 {
            return None;
        }
        self.total_turn_angle().map(|total| total / (self.len() - 2) as f32)
    }
}

pub trait Area {
    ///
    /// judge if a point in the area
    ///
    fn contains(&self, p: &Point) -> bool;
}

///
/// represent a rectangle area
///
#[derive(Debug)]
pub struct RecArea {
    pub(crate) min_lat: f32,
    pub(crate) max_lat: f32,
    pub(crate) min_long: f32,
    pub(crate) max_long: f32,
}

/// CircleArea is a circle area which has a center point of circle and a radius
///
/// # Example
///
/// ```
/// use rsgeo::prelude::*;
/// let ca = CircleArea::new(Point::new(25.2,120.5).unwrap(),30_000.0);
/// // create a circle which coc is (25.2,120.5) and radius is 30km
/// ```
#[derive(Debug)]
pub struct CircleArea {
    /// coc is the center of a circle
    pub(crate) coc: Point,
    pub(crate) radius: f32,
}

impl Area for RecArea {
    /// judge if a point in a rectangle area
    ///
    /// # Example
    /// ```
    /// use rsgeo::prelude::{RecArea,Point,Area};
    /// let ra = RecArea::new(35.0, 36.0, 110.0, 115.0).unwrap();
    /// let pa = Point::new(32.0,112.0).unwrap();
    /// let pb = Point::new(35.1,112.0).unwrap();
    /// assert!(!ra.contains(&pa));
    /// assert!(ra.contains(&pb));
    /// ```
    fn contains(&self, p: &Point) -> bool {
        p.lat < self.max_lat && p.lat > self.min_lat
            && p.long < self.max_long && p.long > self.min_long
    }
}

impl RecArea {
    ///
    /// Create a new RecArea.The latitude and longitude should be valid,otherwise.
    ///
    pub fn new(min_lat: f32,
               max_lat: f32,
               min_long: f32,
               max_long: f32) -> Result<Self, &'static str> {
        if !(valid_lat(min_lat) && valid_lat(max_lat)) {
            Err("Invalid latitude!")
        } else if !(valid_long(min_long) && valid_long(max_long)) {
            Err("Invalid longitude!")
        } else {
            Ok(Self {
                min_lat,
                max_lat,
                min_long,
                max_long,
            })
        }
    }
}

impl Area for CircleArea {
    /// Judge if a point in a rectangle area
    ///
    /// # Example
    /// ```
    /// use rsgeo::prelude::{CircleArea,Point,Area};
    /// let ca = CircleArea::new(Point::new(35.0, 110.0).unwrap(), 350_000.0);
    /// let pa = Point::new(32.0, 112.0).unwrap();
    /// let pb = Point::new(35.1, 112.0).unwrap();
    /// assert!(!ca.contains(&pa));
    /// assert!(ca.contains(&pb));
    /// ```
    fn contains(&self, p: &Point) -> bool {
        self.coc.distance(p) <= self.radius
    }
}

impl CircleArea {
    // Create a new CircleArea
    pub fn new(coc: Point, radius: f32) -> Self {
        Self {
            coc,
            radius,
        }
    }
}

/// Polygon is a vector of points,which points should be closed.
/// (The first and the last point should be same.)
///
/// The polygon can be convex or concave.
#[derive(Debug)]
pub struct Polygon {
    pub(crate) points: Vec<Point>
}

impl Polygon {
    ///
    /// Create a new `polygon`.
    ///
    /// But the rest of points should not be same.
    ///
    /// # Example
    /// ```
    /// use rsgeo::prelude::{Point,Polygon};
    /// let pa = Point::new(32.0,112.0).unwrap();
    /// let pb = Point::new(35.1,112.0).unwrap();
    /// let pc = Point::new(35.3,112.5).unwrap();
    /// let pg = Polygon::new(vec![pa,pb,pc]);
    /// assert!(pg.is_err());
    /// let pg = Polygon::new(vec![pa,pb,pc,pa]);
    /// assert_eq!(pg.unwrap().len(),4);
    /// ```
    pub fn new(points: Vec<Point>) -> Result<Self, &'static str> {
        if points.len() < 4 { return Err("Polygon must have 4 points at least"); }
        if points.last().unwrap() != points.first().unwrap() { return Err("Polygon should be closed,which the first point equals the last point"); }
        let mut hs: HashSet<Point> = HashSet::with_capacity(points.len());
        for i in 0..points.len() - 1 {
            if hs.get(&points.get(i).unwrap()).is_some() { return Err("Polygon has the same point!"); }
        }
        Ok(Self {
            points
        })
    }

    /// Returns the number of elements in the polygon
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// judge if a point is in the polygon area
    ///
    /// # Example
    /// ```
    /// use rsgeo::prelude::{Point,Polygon};
    /// let pa = Point::new(32.0,112.0).unwrap();
    /// let pb = Point::new(35.1,112.0).unwrap();
    /// let pc = Point::new(35.3,112.5).unwrap();
    /// let pg = Polygon::new(vec![pa,pb,pc,pa]).unwrap();
    /// let pt1 =  Point::new(35.0,112.2).unwrap();
    /// let pt2 =  Point::new(34.29,110.3).unwrap();
    /// assert!(pg.contains(&pt1));
    /// assert!(!pg.contains(&pt2));
    /// ```
    pub fn contains(&self, p: &Point) -> bool {
        let mut cross_times = 0;
        let points = self.points.as_slice();
        for i in 0..points.len() - 1 {
            let p1 = &points[i];
            let p2 = &points[i + 1];
            if (p1.lat - p2.lat).abs() < 1e-6 { continue; }
            if p.lat < f32::min(p1.lat, p2.lat) || p.lat >= f32::max(p1.lat, p2.lat) { continue; }
            let x = (p.lat - p1.lat) * (p2.long - p1.long) / (p2.lat - p1.lat) + p1.long;
            if x > p.long {
                cross_times += 1;
            }
        }
        cross_times % 2 == 1
    }

    /// Returns true if the Polygon contains no elements.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn area(&self) -> f32 {
        if self.len() < 4 {
            return 0.0;
        }

        const EARTH_RADIUS: f32 = 6378135.0; // 地球半径（米）
        let mut area = 0.0;
        
        // 遍历多边形的所有边（除了最后一条闭合边）
        for i in 0..self.points.len() - 1 {
            let p1 = &self.points[i];
            let p2 = &self.points[i + 1];
            
            // 转换为弧度
            let lat1 = radians(p1.lat);
            let lon1 = radians(p1.long);
            let lat2 = radians(p2.lat);
            let lon2 = radians(p2.long);
            
            // 使用Girard's theorem计算球面多边形的面积
            // 计算经度差
            let dlon = lon2 - lon1;
            
            // 计算该边对总面积的贡献
            area += (lat2.sin() - lat1.sin()) * dlon;
        }
        
        // 计算最终面积（平方米）
        area = area.abs() * EARTH_RADIUS * EARTH_RADIUS / 2.0;
        
        area
    }
}

pub(crate) fn example_trajectory() -> Trajectory {
    let loc1 = Location::new(Point::new(25.11, 120.98).unwrap(), 0);
    let loc2 = Location::new(Point::new(25.11, 121.1).unwrap(), 7200);
    let loc3 = Location::new(Point::new(25.11, 121.3).unwrap(), 14400);
    let mut t = Trajectory::with_capacity(3);
    t.push_location(&loc1);
    t.push_location(&loc2);
    t.push_location(&loc3);
    t
}

mod test {
    use crate::prelude::{Point, radians, degree, Location, RecArea, Area, CircleArea, Polygon, Trajectory};

    #[test]
    fn test_temp() {}
}
