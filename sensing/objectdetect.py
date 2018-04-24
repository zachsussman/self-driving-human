import math

def dist(p1, p2):
    return math.sqrt((p2['x'] - p1['x']) ** 2 + (p2['y'] - p1['y']) ** 2)

def dot(p1, p2):
    return p1['x'] * p2['x'] + p1['y'] * p2['y']

def mag(p):
    return math.sqrt(dot(p, p))

def sum(p1, p2):
    return {
        "x": p1['x'] + p2['x'],
        "y": p1['y'] + p2['y']
    }

def diff(p1, p2):
    return {
        "x": p1['x'] - p2['x'],
        "y": p1['y'] - p2['y']
    }

def mult(p, s):
    return {
        "x": p['x'] * s,
        "y": p['y'] * s
    }

def normalize(p):
    return mult(p, 1.0/mag(p))

def segment(samples):
    result = []
    curr_segment = []
    for sample in samples:
        if sample['quality'] == 0 or sample['distance'] == 0:
            if len(curr_segment) > 0:
                result.append(curr_segment)
            curr_segment = []
            continue
        curr = {
            "x": sample['distance'] * math.sin(sample['angle'] * math.pi / 180),
            "y": sample['distance'] * math.cos(sample['angle'] * math.pi / 180)
        }
        if len(curr_segment) > 0:
            prev = curr_segment[-1]
            d = dist(prev, curr)
            if d > (sample['distance'] * math.sin(4.0 * math.pi / 180)) * 5:
                # distance between points is more than 5 times the arc length
                result.append(curr_segment)
                curr_segment = []
        curr_segment.append(curr)
    result.append(curr_segment)
    return result

def simplify(segments):
    i = 0
    while i < len(segments):
        seg = segments[i]
        if len(seg) <= 1:
            segments.pop(i)
            continue
        to_remove = []
        for j in range(1, len(seg) - 1):
            prev = seg[j-1]
            curr = seg[j]
            next = seg[j+1]
            normal = normalize({
                'x': next['y'] - prev['y'],
                'y': -(next['x'] - prev['x'])
            })
            if abs(dot(diff(curr, prev), normal)) < mag(diff(next, prev))/10:
                to_remove.append(j)
        to_remove.reverse()
        for j in to_remove:
            seg.pop(j)
        i += 1

def close(segments):
    for seg in segments:
        if len(seg) == 2:
            # create a rectangle from this segment
            normal = normalize({
                'x': seg[1]['y'] - seg[0]['y'],
                'y': -(seg[1]['x'] - seg[0]['x'])
            })
            # ensure it points away from 0,0
            if dot(normal, seg[0]) < 0:
                normal = mult(normal, -1)
            seg.append(sum(seg[1], mult(normal, 500)))
            seg.append(sum(seg[0], mult(normal, 500)))
        else:
            # If the "if" case below holds, we want to do this again so we end
            # up in the "else" case the second time.
            for iter in range(2):
                dir = normalize(diff(seg[-1], seg[-2]))
                normal = normalize({
                    'x': seg[0]['y'] - seg[1]['y'],
                    'y': -(seg[0]['x'] - seg[1]['x'])
                })
                t = dot(diff(seg[-1], seg[0]), dir) / dot(normal, dir)
                point = sum(seg[0], mult(normal, t))
                if mag(point) < mag(mult(sum(seg[0], seg[-1]), 0.5)):
                    # the segment is concave; make it convex and then repeat
                    p1 = diff(seg[0], mult(normal, t))
                    t2 = mag(diff(point, seg[-1]))
                    n2 = normalize({
                        'x': -(seg[-1]['y'] - seg[-2]['y']),
                        'y': seg[-1]['x'] - seg[-2]['x']
                    })
                    p2 = sum(seg[-1], mult(n2, t2))
                    seg.insert(0, p1)
                    seg.append(p2)
                else:
                    # join the ends of the convex segment
                    seg.append(point)
                    break

def detect_polygons(samples):
    segments = segment(samples)
    simplify(segments)
    close(segments)
    return segments
