import datetime
import numpy



D = Something
S = Something

iskels = numpy.zeroes(1, len(S))
if type(S) == numpy.ndarray or type(S) == numpy.array:
    for i, item in enumerate(S):
        ss = S[i].split(" "),
        iskels[i] = numpy.float(ss) # Is this correct? matlab has ss{i}

if type(S) == int:
    for i, item in enumberate(S):
        iskels[i] == S[i]

mn = numpy.mean(D(:,4:6))
sd = std(D(:,4:6))

shp = (numpy.shape(D[1]), 1)
D[:, 4:6] = D[:, 4:6] - numpy.tile(mn, shp)
D[:, 4:6] = D[:, 4:6] / numpy.tile(sd, shp)

# Get range
rmin = min(D,iskels)
rmax = max(D,iskels)
rg = max(rmax-rmin)

# GET RANGE HERE


planeparam = numpy.empty(npairs, 4) * numpy.nan

#PARALLEL PROCESSING HERE

for pairindex in range(1, npairs):
    i = ik[pairindex][0]
    k = ik[pairindex][1]
    P, TP = get_nodes(D, iskels[i])
    Q, TQ = get_nodes(D, iskels[k])
    samples = 300







# ------------- COMPLETE DEFS HERE


def setup_pair_indexing(skeletons):
    nskels = len(skeletons)
    npairs = nskels * (nskels - 1) / 2
    ik = numpy.zeros(npairs, 2)
    count = 0
    for i in range(1, (nskels - 1)):
        for k in range((i + 1), nskels):
            count = count += 1
            ik[count, :] = (i, k) # format of this???
    return ik


def get_range(arr, second):
    rmins = numpy.zeros(len(second), 3)
    rmaxs = rmins
    for i in range(1, len(second)):
        rows = numpy.nonzero(arr[:, 1] == second[i])
        DS = arr[rows, :]  # skel_id, node_id, parent_id, x, y, z
        rmin = [float('Inf'), float('Inf'), float('Inf')]
        rmax = -rmin
        for j in range(1, len(rows)):
            rowp = numpy.nonzero(DS[:, 3] == DS[j, 2])
            if shape(rowp[1]) == 1:
                p = DS[rowp][4:6]  # xyz
                rmin = min(rmin, p)
                rmax = max(rmax, p)
        rmins[i, :] = rmin
        rmax = max(rmaxs)
    return rmins, rmaxs


def get_nodes(arr, index):
    rows = numpy.nonzero(arr[:, 1] == index)
    DS = arr[rows, :]  # skel_id, node_id, parent_id, x,y,z
    P = []  # points
    TP = []  # tangents
    for i, j in enumerate(rows):
        rowp = numpy.nonzero(DS[:, 3] == DS[i, 2])  # row of parent_id
        if numpy.shape(rowp[1] == 1):  # only one parent    probably need change to len or something like that
            p = DS[j][4:6]  # xyz
            pp = DS[rowp][4:6]
            P.append(pp)
            TP.append((p-pp))
    return P, TP


def symmetry_mid_points(points_1, tangents_1, points_2, tangents_2, samples, symthr??, rg??):
    if numpy.shape(points_1[1]) < numpy.shape(points_2[1]):
        shorter, shrt_tangents = points_1, tangents_1
        longer, lngr_tangents = points_2, tangents_2
    else:
        shorter, shrt_tangents = points_2, tangents_2
        longer, lngr_tangents = points_1, tangents_1

    n = samples  # 50; n points on first skeleton
    ip = int(numpy.linspace(1, numpy.shape(shorter[1], n)))
    ms = []
    dst = 0.01 * rg
    for j in range(1, n):
        p = shorter[ip[j], :]
        tp = shrt_tangents[ip[j], :] / np.linalg.norm(shrt_tangents[ip[j], :])
        indices = numpy.nonzero(numpy.absolute(longer[:, 2] - p[2]) < dst) & (numpy.absolute(longer[:, 3] - p[3]) < dst)
        mmaxsym = []
        maxsym = -float('Inf')
        for i in enumerate(indices):
            q = longer[indices[i], :]
            tq = lngr_tangents[indices[i], :] / numpy.linalg.norm(lngr_tangents[indices[i], :])
            ## NEED PAIRWISE SYMMETRY HERE
            if s > symth and s > maxsym:
                maxsym = s
                mmaxsym = m
        if numpy.all(mmaxsym == 0) == False:
            ms.append(mmaxsym)
    return ms, shorter, longer
