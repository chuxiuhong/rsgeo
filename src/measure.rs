use super::prelude::*;


fn stlc_spa(t1: &Trajectory, t2: &Trajectory) -> f32 {
    let mut spa = 0f32;
    for location in &t1.locations {
        spa += (0.0 - location.pos.shortest_distance_from_trajectory(t2).unwrap()).exp();
    }
    spa /= t1.len() as f32;
    spa
}

fn stlc_tem(t1: &Trajectory, t2: &Trajectory) -> f32 {
    let mut tem = 0f32;
    for location in &t1.locations {
        tem += (0.0 - location.time_shortest_distance_from_trajectory(t2).unwrap() as f32).exp();
    }
    tem /= t1.len() as f32;
    tem
}

/// STLC(Spatiotemporal linear combin distance) algorithm
///
/// It is used for compare two trajectories,get similarity in (0,1]
///
/// `lambda` is the weight of spatial,which similarity = lambda * spatial_similarity + (1-lambda) * temporal_similarity
///
/// # Example
///
/// ```
/// use rsgeo::prelude::*;
/// use rsgeo::measure::stlc_trajectory_similarity;
/// let loc1 = Location::new(Point::new(25.11,120.98).unwrap(),0);
/// let loc2 = Location::new(Point::new(26.2,121.1).unwrap(),7200);
/// let loc3 = Location::new(Point::new(26.3,121.3).unwrap(),14400);
/// let mut t = Trajectory::from(vec![loc1,loc2,loc3].as_slice()).unwrap();
/// assert!((stlc_trajectory_similarity(&t, &t, 0.5).unwrap()-1.0).abs() < 1e-6);
/// ```
pub fn stlc_trajectory_similarity(traj1: &Trajectory, traj2: &Trajectory, lambda: f32) -> Option<f32> {
    if traj1.is_empty() || traj2.is_empty() { return None; }
    let spa = (stlc_spa(traj1, traj2) + stlc_spa(traj2, traj1)) / 2.0;
    let tem = (stlc_tem(traj1, traj2) + stlc_tem(traj2, traj1)) / 2.0;
    let sim = spa * lambda + (1.0 - lambda) * tem;
    Some(sim)
}


mod test {

    #[test]
    fn test_temp() {

    }
}