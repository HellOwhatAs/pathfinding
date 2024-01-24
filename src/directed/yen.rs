//! Compute k-shortest paths using [Yen's search
//! algorithm](https://en.wikipedia.org/wiki/Yen%27s_algorithm).
use num_traits::Zero;
use std::cmp::Ordering;
use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::collections::HashSet;
use std::hash::Hash;
use std::iter::FromIterator;

use super::dijkstra::dijkstra_internal;
use super::dijkstra::dijkstra_internal_eid;

/// A representation of a path.
#[derive(Eq, PartialEq, Debug)]
struct Path<N: Eq + Hash + Clone, C: Zero + Ord + Copy> {
    /// The nodes along the path
    nodes: Vec<N>,
    /// The total cost of the path
    cost: C,
}

impl<N, C> PartialOrd for Path<N, C>
where
    N: Eq + Hash + Clone,
    C: Zero + Ord + Copy,
{
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<N, C> Ord for Path<N, C>
where
    N: Eq + Hash + Clone,
    C: Zero + Ord + Copy,
{
    fn cmp(&self, other: &Self) -> Ordering {
        // Compare costs first, then amount of nodes
        let cmp = self.cost.cmp(&other.cost);
        match cmp {
            Ordering::Equal => self.nodes.len().cmp(&other.nodes.len()),
            _ => cmp,
        }
    }
}




#[derive(Eq, PartialEq, Debug)]
struct PathEid<EID: Eq + Hash + Copy, C: Zero + Ord + Copy> {
    /// The nodes along the path
    edges: Vec<EID>,
    /// The total cost of the path
    cost: C,
}

impl<EID, C> PartialOrd for PathEid<EID, C>
where
    EID: Eq + Hash + Copy,
    C: Zero + Ord + Copy,
{
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<EID, C> Ord for PathEid<EID, C>
where
    EID: Eq + Hash + Copy,
    C: Zero + Ord + Copy,
{
    fn cmp(&self, other: &Self) -> Ordering {
        // Compare costs first, then amount of nodes
        let cmp = self.cost.cmp(&other.cost);
        match cmp {
            Ordering::Equal => self.edges.len().cmp(&other.edges.len()),
            _ => cmp,
        }
    }
}



/// Compute the k-shortest paths using the [Yen's search
/// algorithm](https://en.wikipedia.org/wiki/Yen%27s_algorithm).
///
/// The `k`-shortest paths starting from `start` up to a node for which `success` returns `true`
/// are computed along with their total cost. The result is return as a vector of (path, cost).
///
/// - `start` is the starting node.
/// - `successors` returns a list of successors for a given node, along with the cost of moving from
///    the node to the successor. Costs MUST be positive.
/// - `success` checks weather the goal has been reached.
/// - `k` is the amount of paths requests, including the shortest one.
///
/// The returned paths include both the start and the end node and are ordered by their costs
/// starting with the lowest cost. If there exist less paths than requested, only the existing
/// ones (if any) are returned.
///
/// # Example
/// We will search the 3 shortest paths from node C to node H. See
/// <https://en.wikipedia.org/wiki/Yen's_algorithm#Example> for a visualization.
///
/// ```
/// use pathfinding::prelude::yen;
/// // Find 3 shortest paths from 'c' to 'h'
/// let paths = yen(
///     &'c',
///     |c| match c {
///         'c' => vec![('d', 3), ('e', 2)],
///         'd' => vec![('f', 4)],
///         'e' => vec![('d', 1), ('f', 2), ('g', 3)],
///         'f' => vec![('g', 2), ('h', 1)],
///         'g' => vec![('h', 2)],
///         'h' => vec![],
///         _ => panic!(""),
///         },
///         |c| *c == 'h',
/// 3);
/// assert_eq!(paths.len(), 3);
/// assert_eq!(paths[0], (vec!['c', 'e', 'f', 'h'], 5));
/// assert_eq!(paths[1], (vec!['c', 'e', 'g', 'h'], 7));
/// assert_eq!(paths[2], (vec!['c', 'd', 'f', 'h'], 8));
///
/// // An example of a graph that has no path from 'c' to 'h'.
/// let empty = yen(
///     &'c',
///     |c| match c {
///         'c' => vec![('d', 3)],
///         'd' => vec![],
///         _ => panic!(""),
///     },
///     |c| *c == 'h',
///     2);
/// assert!(empty.is_empty());
/// ```

pub fn yen<N, C, FN, IN, FS>(
    start: &N,
    mut successors: FN,
    mut success: FS,
    k: usize,
) -> Vec<(Vec<N>, C)>
where
    N: Eq + Hash + Clone,
    C: Zero + Ord + Copy,
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, C)>,
    FS: FnMut(&N) -> bool,
{
    let Some((n, c)) = dijkstra_internal(start, &mut successors, &mut success) else {
        return vec![];
    };

    let mut visited = HashSet::new();
    // A vector containing our paths.
    let mut routes = vec![Path { nodes: n, cost: c }];
    // A min-heap to store our lowest-cost route candidate
    let mut k_routes = BinaryHeap::new();
    for ki in 0..(k - 1) {
        if routes.len() <= ki || routes.len() == k {
            // We have no more routes to explore, or we have found enough.
            break;
        }
        // Take the most recent route to explore new spurs.
        let previous = &routes[ki].nodes;
        // Iterate over every node except the sink node.
        for i in 0..(previous.len() - 1) {
            let spur_node = &previous[i];
            let root_path = &previous[0..i];

            let mut filtered_edges = HashSet::new();
            for path in &routes {
                if path.nodes.len() > i + 1
                    && &path.nodes[0..i] == root_path
                    && &path.nodes[i] == spur_node
                {
                    filtered_edges.insert((&path.nodes[i], &path.nodes[i + 1]));
                }
            }
            let filtered_nodes: HashSet<&N> = HashSet::from_iter(root_path);
            // We are creating a new successor function that will not return the
            // filtered edges and nodes that routes already used.
            let mut filtered_successor = |n: &N| {
                successors(n)
                    .into_iter()
                    .filter(|(n2, _)| {
                        !filtered_nodes.contains(&n2) && !filtered_edges.contains(&(n, n2))
                    })
                    .collect::<Vec<_>>()
            };

            // Let us find the spur path from the spur node to the sink using.
            if let Some((spur_path, _)) =
                dijkstra_internal(spur_node, &mut filtered_successor, &mut success)
            {
                let nodes: Vec<N> = root_path.iter().cloned().chain(spur_path).collect();
                // If we have found the same path before, we will not add it.
                if !visited.contains(&nodes) {
                    // Since we don't know the root_path cost, we need to recalculate.
                    let cost = make_cost(&nodes, &mut successors);
                    let path = Path { nodes, cost };
                    // Mark as visited
                    visited.insert(path.nodes.clone());
                    // Build a min-heap
                    k_routes.push(Reverse(path));
                }
            }
        }
        if let Some(k_route) = k_routes.pop() {
            let route = k_route.0;
            let cost = route.cost;
            routes.push(route);
            // If we have other potential best routes with the same cost, we can insert
            // them in the found routes since we will not find a better alternative.
            while routes.len() < k {
                let Some(k_route) = k_routes.peek() else {
                    break;
                };
                if k_route.0.cost == cost {
                    let Some(k_route) = k_routes.pop() else {
                        break; // Cannot break
                    };
                    routes.push(k_route.0);
                } else {
                    break; // Other routes have higher cost
                }
            }
        }
    }

    routes.sort_unstable();
    routes
        .into_iter()
        .map(|Path { nodes, cost }| (nodes, cost))
        .collect()
}

/// 
pub fn yen_eid<N, C, FN, IN, FS, EID, EFN>(
    start: &N,
    mut successors: FN,
    mut eid2uvc: EFN,
    mut success: FS,
    k: usize,
) -> Vec<(Vec<EID>, C)>
where
    N: Eq + Hash + Clone,
    C: Zero + Ord + Copy,
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, C, EID)>,
    FS: FnMut(&N) -> bool,
    EID: Copy + Eq + Hash,
    EFN: FnMut(EID) -> (N, N, C)
{
    let Some((n, c)) = dijkstra_internal_eid(start, &mut successors, &mut success) else {
        return vec![];
    };

    let mut visited = HashSet::new();
    // A vector containing our paths.
    let mut routes = vec![PathEid { edges: n, cost: c }];
    // A min-heap to store our lowest-cost route candidate
    let mut k_routes = BinaryHeap::new();
    for ki in 0..(k - 1) {
        if routes.len() <= ki || routes.len() == k {
            // We have no more routes to explore, or we have found enough.
            break;
        }
        // Take the most recent route to explore new spurs.
        let previous = &routes[ki].edges;
        // Iterate over every node except the sink node.
        for i in 0..previous.len() {
            let spur_node = &eid2uvc(previous[i]).0;
            let root_path = &previous[0..i];

            let mut filtered_edges = HashSet::new();
            for path in &routes {
                if path.edges.len() > i
                    && &path.edges[0..i] == root_path
                {
                    filtered_edges.insert(path.edges[i]);
                }
            }
            let filtered_nodes_tmp: Vec<N> = root_path.iter().map(|eid| eid2uvc(*eid).0).collect();
            let filtered_nodes: HashSet<&N> = HashSet::from_iter(filtered_nodes_tmp.iter().map(|n| n));
            // We are creating a new successor function that will not return the
            // filtered edges and nodes that routes already used.
            let mut filtered_successor = |n: &N| {
                successors(n)
                    .into_iter()
                    .filter(|(n2, _, eid)| {
                        !filtered_nodes.contains(&n2) && !filtered_edges.contains(eid)
                    })
                    .collect::<Vec<_>>()
            };

            // Let us find the spur path from the spur node to the sink using.
            if let Some((spur_path, _)) =
                dijkstra_internal_eid(spur_node, &mut filtered_successor, &mut success)
            {
                let edges: Vec<EID> = root_path.iter().cloned().chain(spur_path).collect();
                // If we have found the same path before, we will not add it.
                if !visited.contains(&edges) {
                    // Since we don't know the root_path cost, we need to recalculate.
                    let cost = make_cost_eid(&edges, &mut eid2uvc);
                    let path = PathEid { edges, cost };
                    // Mark as visited
                    visited.insert(path.edges.clone());
                    // Build a min-heap
                    k_routes.push(Reverse(path));
                }
            }
        }
        if let Some(k_route) = k_routes.pop() {
            let route = k_route.0;
            let cost = route.cost;
            routes.push(route);
            // If we have other potential best routes with the same cost, we can insert
            // them in the found routes since we will not find a better alternative.
            while routes.len() < k {
                let Some(k_route) = k_routes.peek() else {
                    break;
                };
                if k_route.0.cost == cost {
                    let Some(k_route) = k_routes.pop() else {
                        break; // Cannot break
                    };
                    routes.push(k_route.0);
                } else {
                    break; // Other routes have higher cost
                }
            }
        }
    }

    routes.sort_unstable();
    routes
        .into_iter()
        .map(|PathEid { edges, cost }| (edges, cost))
        .collect()
}

fn make_cost<N, FN, IN, C>(nodes: &[N], successors: &mut FN) -> C
where
    N: Eq,
    C: Zero,
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, C)>,
{
    let mut cost = C::zero();
    for edge in nodes.windows(2) {
        for (n, c) in successors(&edge[0]) {
            if n == edge[1] {
                cost = cost + c;
            }
        }
    }
    cost
}

fn make_cost_eid<N, C, EID, EFN>(edges: &[EID], eid2uvc: &mut EFN) -> C
where
    C: Zero,
    EID: Copy + Eq + Hash,
    EFN: FnMut(EID) -> (N, N, C)
{
    let mut cost = C::zero();
    for edge in edges {
        cost = cost + eid2uvc(*edge).2;
    }
    cost
}


#[test]
fn test_yen_eid_1() {
    let paths = yen_eid(
        &'a',
        |c| match c {
            'a' => vec![('b', 3, 0), ('b', 2, 1)],
            'b' => vec![],
            _ => panic!(""),
            },
            |eid| match eid {
                0 => ('a', 'b', 3),
                1 => ('a', 'b', 2),
                _ => unreachable!()
            },
            |c| *c == 'b',
    3000);
    assert_eq!(paths, vec![(vec![1], 2), (vec![0], 3)]);
}

#[test]
fn test_yen_eid_2() {
    let paths = yen(
        &'c',
        |c| match c {
            'c' => vec![('d', 3), ('e', 2)],
            'd' => vec![('f', 4)],
            'e' => vec![('d', 1), ('f', 2), ('g', 3)],
            'f' => vec![('g', 2), ('h', 1)],
            'g' => vec![('h', 2)],
            'h' => vec![],
            _ => panic!(""),
            },
            |c| *c == 'h',
    usize::MAX);
    let paths_eid = yen_eid(
        &'c',
        |c| match c {
            'c' => vec![('d', 3, "cd"), ('e', 2, "ce")],
            'd' => vec![('f', 4, "df")],
            'e' => vec![('d', 1, "ed"), ('f', 2, "ef"), ('g', 3, "eg")],
            'f' => vec![('g', 2, "fg"), ('h', 1, "fh")],
            'g' => vec![('h', 2, "gh")],
            'h' => vec![],
            _ => panic!(""),
            },
        |eid| match eid {
            "cd" => ('c', 'd', 3),
            "ce" => ('c', 'e', 2),
            "df" => ('d', 'f', 4),
            "ed" => ('e', 'd', 1),
            "ef" => ('e', 'f', 2),
            "eg" => ('e', 'g', 3),
            "fg" => ('f', 'g', 2),
            "fh" => ('f', 'h', 1),
            "gh" => ('g', 'h', 2),
            _ => unreachable!()
        },
        |c| *c == 'h',
        usize::MAX
    );
    // assert_eq!(paths.into_iter().map(|e|e), )
    let p = paths.into_iter().map(|(path, l)| (path.windows(2).map(|e| format!("{}{}", e[0], e[1])).collect::<Vec<_>>(), l)).collect::<Vec<_>>();
    let p_eid = paths_eid.into_iter().map(|(path, l)| (path.into_iter().map(|e| e.to_string()).collect::<Vec<_>>(), l)).collect::<Vec<_>>();
    assert_eq!(p, p_eid);
}