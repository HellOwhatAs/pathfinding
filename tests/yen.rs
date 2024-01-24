use pathfinding::prelude::{yen, yen_eid};

// A simple tests of Yen's algorithm based on the example and visualization
// from https://en.wikipedia.org/wiki/Yen's_algorithm#Example.
#[test]
fn simple() {
    let result = yen(
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
        3,
    );

    assert_eq!(result.len(), 3);
    assert_eq!(result[0], (vec!['c', 'e', 'f', 'h'], 5));
    assert_eq!(result[1], (vec!['c', 'e', 'g', 'h'], 7));
    assert_eq!(result[2], (vec!['c', 'd', 'f', 'h'], 8));
}

/// Tests that we correctly return fewer routes when
/// we exhaust all possible paths.
#[test]
fn ask_more_than_exist() {
    let result = yen(
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
        10,
    );

    // we asked for 10 but the graph can only produce 7
    assert_eq!(
        result.iter().map(|&(_, c)| c).collect::<Vec<_>>(),
        vec![5, 7, 8, 8, 8, 11, 11]
    );
}

/// Test that we return None in case there is no solution
#[test]
fn no_path() {
    let result = yen(
        &'c',
        |c| match c {
            'c' => vec![('d', 3), ('e', 2)],
            'd' => vec![('f', 4)],
            'e' => vec![('d', 1), ('f', 2), ('g', 3)],
            'f' => vec![('g', 2), ('d', 1)],
            'g' => vec![('e', 2)],
            'h' => vec![],
            _ => panic!(""),
        },
        |c| *c == 'h',
        2,
    );

    assert!(result.is_empty());
}

/// Test that we support loops
#[test]
fn single_node() {
    let result = yen(
        &'c',
        |c| match c {
            'c' => vec![('c', 1)],
            _ => panic!(""),
        },
        |c| *c == 'c',
        2,
    );

    assert_eq!(result, vec![(vec!['c'], 0)]);
}

/// Test that we don't panic if an alternative path is more than two nodes longer than a previous one.
#[test]
fn longer_alternative_path() {
    let result = yen(
        &'c',
        |c| match c {
            'c' => vec![('d', 1), ('h', 1)],
            'd' => vec![('e', 1)],
            'e' => vec![('f', 1)],
            'f' => vec![('g', 1), ('h', 1)],
            'g' => vec![('h', 1)],
            'h' => vec![],
            _ => panic!(""),
        },
        |c| *c == 'h',
        3,
    );

    assert_eq!(result.len(), 3);
    assert_eq!(result[0], (vec!['c', 'h'], 1));
    assert_eq!(result[1], (vec!['c', 'd', 'e', 'f', 'h'], 4));
    assert_eq!(result[2], (vec!['c', 'd', 'e', 'f', 'g', 'h'], 5));
}

/// Check that we return all loopless paths
/// (issue #467)
#[test]
fn all_paths() {
    let mut result = yen(
        &'a',
        |c| match c {
            'a' => vec![('b', 1), ('c', 1), ('d', 1)],
            'b' => vec![('c', 1), ('d', 1)],
            'c' => vec![('b', 1), ('d', 1)],
            'd' => vec![],
            _ => unreachable!(),
        },
        |c| *c == 'd',
        usize::MAX,
    );
    result.sort_unstable();
    assert_eq!(
        result,
        vec![
            (vec!['a', 'b', 'c', 'd'], 3),
            (vec!['a', 'b', 'd'], 2),
            (vec!['a', 'c', 'b', 'd'], 3),
            (vec!['a', 'c', 'd'], 2),
            (vec!['a', 'd'], 1),
        ]
    );
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