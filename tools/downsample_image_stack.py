#!/usr/bin/env python
import imghdr
import numpy
import os
import pickle
import scipy.misc
import scipy.ndimage
import sys


#: scipy.version.full_version
#: '0.15.1'
#
#: numpy.version.full_version
#: '1.9.2'

dimdir = '/zflode/130201zf142/Serving/160515_SWiFT_60nmpx_singles/'
doutdir = '/zflode/130201zf142/Serving/160515_SWiFT_60nmpx_singles_300iso/'
imagepickle = '/zflode/Dropbox/Code/hildebrand16/tools/pickle_test_300.p'

inres = numpy.array([56.4, 56.4, 60.])
outres = numpy.array([300., 300., 300.])
scale = inres / outres

secsize = int(numpy.ceil(1 / scale[2]))
default_slice_buff = int(numpy.ceil(1/scale[2]))


def getimageinfo(imdir):
    print 'Retrieving image info.  This may take some time.'
    imageinfo = {}
    global shape
    shape = None
    for fil in os.listdir(imdir):
        path = imdir + fil
        if imghdr.what(path):
            root, ext = os.path.splitext(fil)
            slc = slicefromroot(root)
            imageinfo[slc] = {'path': imdir,
                              'image': fil,
                              'extension': ext,
                              'fullpath': path}
            if shape is None:
                shape = scipy.misc.imread(path).shape
    return imageinfo


def slicefromroot(root):
    # return int(root)
    return int(root.rstrip('T'))


def buildarray(images, minim, maxim, max_slice_buff=default_slice_buff):
    slice_buff = maxim - minim
    if (slice_buff > max_slice_buff):
        raise Exception('distance between {} and {} too great'
                        'for buffer of size {} images.'.format(minim,
                                                               maxim,
                                                               max_slice_buff))
    stack = numpy.empty([shape[0], shape[1], slice_buff]) * numpy.nan
    for slc in range(minim, (maxim)):
        if slc in images.keys():
            stack[:, :, (slc - minim)] = scipy.misc.imread(images[slc]
                                                           ['fullpath'])
    print "array built!"
    return stack


def save_images(img, start, zscale=scale[2]):
    if not os.path.exists(outdir):
        os.makedirs(outdir)
    for zcoord in range(len(img[0, 0])):
        fn = outdir + '{}T_down.PGM'.format(str(int(start + (float(zcoord) *
                                                1. / zscale))).zfill(5))
        if numpy.std(img[:, :, zcoord]) < 10:
            print "WARNING!  Low standard deviation on {}".format(fn)
        scipy.misc.imsave(fn, img[:, :, zcoord])


def intrazpix(stack, secperpix):
    scaledshape = (scale * numpy.array(stack.shape))
    '''
    stack = numpy.ma.masked_invalid(stack)
    stack.data = numpy.nan_to_num(stack.data)
    newstack = scipy.ndimage.zoom(stack.data,
                                  [scale[0], scale[1], 1.],
                                  order=1)
    newmask = numpy.resize(stack.mask, newstack.shape)
    del stack
    stack = numpy.ma.array(newstack, mask=newmask)
    del newstack
    del newmask
    '''
    for j, subarr in enumerate(numpy.array_split(stack,
                                                 stack.shape[2] / secperpix,
                                                 axis=2)):
        print "averaging array of shape {}".format(subarr.shape)
        substack = numpy.ma.masked_invalid(subarr)
        subzs = numpy.nan_to_num(substack.data)
        newstack = scipy.ndimage.zoom(subzs,
                                      [scale[0], scale[1], 1.],
                                      order=1)
        del subzs
        marray = numpy.ma.array(newstack,
                                mask=numpy.resize(substack.mask, newstack.shape))
        meanarr = numpy.ma.mean(marray, axis=2)
        try:
            newarr[:, :, j] = meanarr
        except:
            newshape = list((marray.shape))
            newshape[2] = int(numpy.ceil(scaledshape[2]))
            newarr = numpy.empty(newshape)
            newarr[:, :, j] = meanarr
        print "Averaged with stdev {}".format(numpy.std(meanarr))
    return newarr


def process_stack(images, slice_buff=default_slice_buff, onlyintra_zpix=True):
    minim = min(images.keys())
    maxim = max(images.keys())

    for i in range(minim, maxim, slice_buff):
        j = i + slice_buff
        print "processing slices {} to {}.".format(i, j)
        stack = buildarray(images, i, j)
        if onlyintra_zpix:
            stack = intrazpix(stack, secsize)
        save_images(stack, i, zscale=scale[2])

if __name__ == "__main__":
    try:
        imdir = sys.argv[1]
        outdir = sys.argv[2]
    except:
        imdir = dimdir
        outdir = doutdir

    if os.path.exists(imagepickle):
        with open(imagepickle, 'r') as p:
            imgs, shape = pickle.load(p)
    else:
        imgs = getimageinfo(imdir)
        with open(imagepickle, 'w') as p:
            pickle.dump([imgs, shape], p)
    process_stack(imgs)
