from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import argparse
import cv2
import math
import numpy as np
import os
from scipy import ndimage
import signal
import sys
import tensorboard as tb
import tensorflow as tf
from tensorflow.python.ops import array_ops
from tensorflow.python.ops import math_ops
from tensorflow.python.ops import nn_ops
from tensorflow.python.ops import nn
# from tensorflow.contrib.framework.python.ops import sort_ops
from tensorflow.python.framework import tensor_util
from tensorflow.python.framework import ops as framework_ops
from tensorflow.python.framework import constant_op
import time as tm
from tensorflow.python.client import timeline
from scipy.ndimage.filters import maximum_filter as max_filt

FLAGS = None

# Default Webcam Colormapping - BGR
cap, writer, edit = None, None, None
graph = tf.Graph()


def open_stream():
    # Uses video stream unless otherwise specified
    if FLAGS.video_stream:
        input_mode = 1
        cap = cv2.VideoCapture(input_mode)
        if not cap.isOpened():
            print('Unable to open video stream ', input_mode)
            return None
    else:
        # Looks for video file inside Flags.data_dir directory
        file_name = os.path.join(FLAGS.data_dir, '*.avi')
        if not os.path.exists(file_name):
            print('Could not find suitable video file in ', FLAGS.data_dir)
            return None
        cap = cv2.VideoCapture(os.path.join(FLAGS.data_dir, '*.avi'))
        if not cap.isOpened():
            print('Video file could not be opened: ', file_name)
            return None
    return cap


def open_src():
    if not os.path.exists('test_img.jpg'):
        print('could not find test_img.jpg')
    else:
        cap = cv2.VideoCapture('test_img.jpg')
        if not cap.isOpened():
            print('Unable to open video stream on test_img.jpg')
            return None
        else:
            return cap


def open_video_writer(cap, record='edited.avi'):
    fourcc = int(cv2.VideoCapture.get(cap, cv2.CAP_PROP_FOURCC))
    fps = int(cv2.VideoCapture.get(cap, cv2.CAP_PROP_FPS))
    width = int(cv2.VideoCapture.get(cap, cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cv2.VideoCapture.get(cap, cv2.CAP_PROP_FRAME_HEIGHT))
    print(fourcc,fps,width,height)
    if not os.path.isdir(FLAGS.data_dir):
        os.makedirs(FLAGS.data_dir)
    if os.path.exists(os.path.join(FLAGS.data_dir, record)):
        os.remove(os.path.join(FLAGS.data_dir, record))
    if not cap.isOpened:
        print('No video stream to record.')
        return None
    writer = cv2.VideoWriter(os.path.join(FLAGS.data_dir, record), cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), fps, (width, height))
    # writer = cv2.VideoWriter(os.path.join(FLAGS.data_dir, record), fourcc, fps, (width, height))
    return writer


def close_stream(cap):
    print('Closing video stream: ...')
    if cap is not None and cap.isOpened():
        cap.release()
        print('video stream closed.')
    else:
        print('already closed.')


def close_video_writer(writer):
    print('Closing recording stream ...')
    if writer is not None and writer.isOpened:
        print('recording stream closed.')
        writer.release()
    else:
        print('already closed.')


def create_graph():
    start = tm.time()
    with graph.as_default():
        input_placeholder = tf.placeholder(tf.uint8,shape=[640,480,3])
        # tf_frame = tf.convert_to_tensor(input_placeholder,np.uint8)
        tf_frame_f32 = tf.image.convert_image_dtype(input_placeholder,tf.float32)
        reshaped = tf.expand_dims(tf_frame_f32,0)
        # print(reshaped)
        sobel = tf.image.sobel_edges(reshaped)
        # fake_sobel = tf.expand_dims(reshaped,4)
        # sobel = tf.tile(fake_sobel,[1,1,1,1,2])
        # print(sobel)
        sob_abs = tf.abs(sobel,'sob_abs')
        sob_squeeze = tf.squeeze(sob_abs)
        sob_img = tf.reduce_max(sob_squeeze,3,False)
        sob_u8 = tf.image.convert_image_dtype(sob_img,dtype=tf.uint8)
        max_val = tf.reduce_max(tf.reduce_max(tf.reduce_max(sob_u8,0),0),0)
        sob_img_u8_norm = tf.scalar_mul(255/max_val,sob_u8)

    # throw_away_test = sess.run(sob_img_u8_norm)

    end = tm.time()
    print(end-start)


def process_image2(frame,sess):
    start = tm.time()
    nd_sobel = cv2.cvtColor(ndimage.sobel(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)),cv2.COLOR_GRAY2RGB)
    print(np.shape(nd_sobel))
    print(nd_sobel)
    return nd_sobel


def kill_signal(signum,frame):
    close_stream(cap)
    close_stream(edit)
    close_stream(writer)
    cv2.destroyAllWindows()
    sys.exit(0)


# def sobel_edges(image,size):
#   """Returns a tensor holding Sobel edge maps.
#   Arguments:
#     image: Image tensor with shape [batch_size, h, w, d] and type float32 or
#     float64.  The image(s) must be 2x2 or larger.
#   Returns:
#     Tensor holding edge maps for each channel. Returns a tensor with shape
#     [batch_size, h, w, d, 2] where the last two dimensions hold [[dy[0], dx[0]],
#     [dy[1], dx[1]], ..., [dy[d-1], dx[d-1]]] calculated using the Sobel filter.
#   """
#   # Define vertical and horizontal Sobel filters.
#   static_image_shape = image.get_shape()
#   image_shape = array_ops.shape(image)
#   kernels = [[[-1, -2, -1], [0, 0, 0], [1, 2, 1]],
#              [[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]]]
#   num_kernels = len(kernels)
#   kernels = np.transpose(np.asarray(kernels), (1, 2, 0))
#   kernels = np.expand_dims(kernels, -2)
#   kernels_tf = constant_op.constant(kernels, dtype=image.dtype)
#
#   kernels_tf = array_ops.tile(kernels_tf, [1, 1, image_shape[-1], 1],
#                               name='sobel_filters')
#
#   # Use depth-wise convolution to calculate edge maps per channel.
#   pad_sizes = [[0, 0], [1, 1], [1, 1], [0, 0]]
#   padded = array_ops.pad(image, pad_sizes, mode='REFLECT')
#
#   # Output tensor has shape [batch_size, h, w, d * num_kernels].
#   strides = [1, 1, 1, 1]
#   output = nn.depthwise_conv2d(padded, kernels_tf, strides, 'VALID')
#
#   # Reshape to [batch_size, h, w, d, num_kernels].
#   shape = array_ops.concat([image_shape, [num_kernels]], 0)
#   output = array_ops.reshape(output, shape=shape)
#   output.set_shape(static_image_shape.concatenate([num_kernels]))
#   return output


def sort(values, axis=-1, direction='ASCENDING', name=None):
    """Sorts a tensor.
  Args:
    values: 1-D or higher numeric `Tensor`.
    axis: The axis along which to sort. The default is -1, which sorts the last
        axis.
    direction: The direction in which to sort the values (`'ASCENDING'` or
        `'DESCENDING'`).
    name: Optional name for the operation.
  Returns:
    A `Tensor` with the same dtype and shape as `values`, with the elements
        sorted along the given `axis`.
  Raises:
    ValueError: If axis is not a constant scalar, or the direction is invalid.
  """
    with framework_ops.name_scope(name, 'sort'):
        return _sort_or_argsort(values, axis, direction, return_argsort=False)


def argsort(values, axis=-1, direction='ASCENDING', stable=False, name=None):
    """Returns the indices of a tensor that give its sorted order along an axis.
  For a 1D tensor, `tf.gather(values, tf.argsort(values))` is equivalent to
  `tf.sort(values)`. For higher dimensions, the output has the same shape as
  `values`, but along the given axis, values represent the index of the sorted
  element in that slice of the tensor at the given position.
  Args:
    values: 1-D or higher numeric `Tensor`.
    axis: The axis along which to sort. The default is -1, which sorts the last
        axis.
    direction: The direction in which to sort the values (`'ASCENDING'` or
        `'DESCENDING'`).
    stable: If True, equal elements in the original tensor will not be
        re-ordered in the returned order. Unstable sort is not yet implemented,
        but will eventually be the default for performance reasons. If you
        require a stable order, pass `stable=True` for forwards compatibility.
    name: Optional name for the operation.
  Returns:
    An int32 `Tensor` with the same shape as `values`. The indices that would
        sort each slice of the given `values` along the given `axis`.
  Raises:
    ValueError: If axis is not a constant scalar, or the direction is invalid.
  """
    del stable  # Unused.
    with framework_ops.name_scope(name, 'argsort'):
        return _sort_or_argsort(values, axis, direction, return_argsort=True)


def _sort_or_argsort(values, axis, direction, return_argsort):
    """Internal sort/argsort implementation.
  Args:
    values: The input values.
    axis: The axis along which to sort.
    direction: 'ASCENDING' or 'DESCENDING'.
    return_argsort: Whether to return the argsort result.
  Returns:
    Either the sorted values, or the indices of the sorted values in the
        original tensor. See the `sort` and `argsort` docstrings.
  Raises:
    ValueError: If axis is not a constant scalar, or the direction is invalid.
  """
    if direction not in _SORT_IMPL:
        raise ValueError('%s should be one of %s' %
                         (direction, ', '.join(sorted(_SORT_IMPL.keys()))))
    # Axis must be an integer, not a Tensor.
    axis = framework_ops.convert_to_tensor(axis, name='axis')
    axis_static = tensor_util.constant_value(axis)
    if axis.shape.ndims != 0 or axis_static is None:
        raise ValueError('axis must be a constant scalar')
    axis_static = int(axis_static)  # Avoids NumPy casting error

    values = framework_ops.convert_to_tensor(values, name='values')

    return _SORT_IMPL[direction](values, axis_static, return_argsort)


def _descending_sort(values, axis, return_argsort=False):
    """Sorts values in reverse using `top_k`.
    Args:
      values: Tensor of numeric values.
      axis: Index of the axis which values should be sorted along.
      return_argsort: If False, return the sorted values. If True, return the
          indices that would sort the values.
    Returns:
      The sorted values.
    """
    k = array_ops.shape(values)[axis]
    rank = array_ops.rank(values)
    static_rank = values.shape.ndims
    # Fast path: sorting the last axis.
    if axis == -1 or axis + 1 == values.get_shape().ndims:
        # print('here1')
        top_k_input = values
        transposition = None
    else:
        # Otherwise, transpose the array. Swap axes `axis` and `rank - 1`.
        if axis < 0:
            # print('here2')
            # Calculate the actual axis index if counting from the end. Use the static
            # rank if available, or else make the axis back into a tensor.
            axis += static_rank or rank
        if static_rank is not None:
            # print('here3')
            # Prefer to calculate the transposition array in NumPy and make it a
            # constant.
            transposition = constant_op.constant(
                np.r_[
                    # Axes up to axis are unchanged.
                    np.arange(axis),
                    # Swap axis and rank - 1.
                    [static_rank - 1],
                    # Axes in [axis + 1, rank - 1) are unchanged.
                    np.arange(axis + 1, static_rank - 1),
                    # Swap axis and rank - 1.
                    [axis]],
                name='transposition')
        else:
            # print('here4')
            # Generate the transposition array from the tensors.
            transposition = array_ops.concat(
                [
                        # Axes up to axis are unchanged.
                        math_ops.range(axis),
                        # Swap axis and rank - 1.
                        [rank - 1],
                        # Axes in [axis + 1, rank - 1) are unchanged.
                        math_ops.range(axis + 1, rank - 1),
                        # Swap axis and rank - 1.
                        [axis]
                ],
                axis=0)
        top_k_input = array_ops.transpose(values, transposition)

    # print('here5')
    values, indices = nn_ops.top_k(top_k_input, k)
    return_value = indices if return_argsort else values
    if transposition is not None:
        # print('here6')
        # transposition contains a single cycle of length 2 (swapping 2 elements),
        # so it is an involution (it is its own inverse).
        return_value = array_ops.transpose(return_value, transposition)
    return return_value


def _ascending_sort(values, axis, return_argsort=False):
    # Negate the values to get the ascending order from descending sort.
    values_or_indices = _descending_sort(-values, axis, return_argsort)
    # If not argsort, negate the values again.
    return values_or_indices if return_argsort else -values_or_indices


_SORT_IMPL = {
    'ASCENDING': _ascending_sort,
    'DESCENDING': _descending_sort,
}


def get_local_maxima(in_tensor,scale):
    in_tensor = tf.expand_dims(in_tensor, 3, name='R_expanded')
    max_pooled_in_tensor = tf.nn.pool(in_tensor, window_shape=(scale, scale), pooling_type='MAX', padding='SAME')
    maxima = tf.where(tf.equal(in_tensor, max_pooled_in_tensor), in_tensor, tf.zeros_like(in_tensor))
    return tf.squeeze(maxima,3)


def compute_M(sobel_single_channel, scale=3, weight=3, step_size=1):
    # dx, dy shape: [1, height, width, 1]
    # kernel shape: [filter_height, filter_width, in_channels, out_channels]
    dy = tf.slice(sobel_single_channel, [0, 0, 0, 0], [-1, -1, -1, 1], name = 'dx')
    dx = tf.slice(sobel_single_channel, [0, 0, 0, 1], [-1, -1, -1, 1], name = 'dy')
    dxdx = tf.square(dx, name='dx_sqr')
    dxdy = tf.multiply(dx, dy, name='dx_dy')
    dydy = tf.square(dy, name='dy_sqr')
    conv_kernel = tf.constant(1.0/weight, tf.float32, [scale, scale, 1, 1])
    M11 = tf.nn.conv2d(dxdx, conv_kernel, [1, step_size, step_size, 1], 'SAME', name='M11')
    M12 = tf.nn.conv2d(dxdy, conv_kernel, [1, step_size, step_size, 1], 'SAME', name='M12')
    M21 = tf.tile(M12, [1, 1, 1, 1], name='M21')
    M22 = tf.nn.conv2d(dydy, conv_kernel, [1, step_size, step_size, 1], 'SAME', name='M22')
    M = tf.squeeze(tf.stack([tf.stack([M11, M12], 3, name='M_top'), tf.stack([M21, M22], 3, name='M_bot')], 3, name='M'), 5)
    # tf.summary.histogram('M',M)
    # output [1, height, width, 2, 2]
    return M, dx, dy


def compute_eigen(sobel_single_channel, scale=4):
    M = compute_M(sobel_single_channel, scale)[0]
    print(M)
    eVal, eVec = tf.self_adjoint_eig(M)
    return eVal, eVec


def compute_R(sobel_single_channel, scale, k):
    M = compute_M(sobel_single_channel, scale, 1)[0]
    det = tf.matrix_determinant(M)
    trace = tf.trace(M)
    # M_det = tf.matrix_determinant(M, name = 'M_det')
    # M_trace = tf.scalar_mul(k, tf.square(tf.trace(M)))
    R = tf.subtract(det, tf.scalar_mul(k, tf.square(trace)), name='R')
    # output with same scale as input
    return R


def find_corners_in_channel(channel, scale, k, mask):
    # Compute corner response at given scale
    # apply hue mask
    # find best corner in channel
    R = tf.squeeze(get_local_maxima(compute_R(channel, scale, k), scale), name='R_max_pooled')
    # tf.summary.histogram('R', R)
    if not mask is None:
        masked = tf.where(mask, R, tf.zeros([480, 640]), name='masked')
    else:
        masked = R

    return masked


def find_top_corners(set):
    values, indices = tf.nn.top_k(set, 1, name='top_corners_at_pixel')
    return tf.squeeze(values, name='pixel_topn_squeeze')


def find_corners(img, scale_set, mask, k=0.1):
    reshaped = tf.expand_dims(img, 0)
    sobel = tf.image.sobel_edges(reshaped)
    sob_r = tf.squeeze(tf.slice(sobel, [0, 0, 0, 0, 0], [-1, -1, -1, 1, -1]), 3, name = 'sob_r')
    sob_g = tf.squeeze(tf.slice(sobel, [0, 0, 0, 1, 0], [-1, -1, -1, 1, -1]), 3, name = 'sob_g')
    sob_b = tf.squeeze(tf.slice(sobel, [0, 0, 0, 2, 0], [-1, -1, -1, 1, -1]), 3, name = 'sob_b')
    # channels = [sob_r, sob_g, sob_b]
    channels = [sob_r,sob_g,sob_b]
    top_n = 2
    corners = tf.constant(0,tf.float32,[480,640])
    for scale in range(scale_set[0], scale_set[1], scale_set[2]):
        corner_scale_subset = tf.constant(0,tf.float32,[480,640])
        for channel in channels:
            R = tf.squeeze(compute_R(channel,scale,k))
            corner_scale_subset = tf.add(corner_scale_subset, R)
        corners = tf.add(corners, corner_scale_subset, name = 'scale_subset_corners_3chan')
    # corners_across_scale = find_top_corners(tf.stack(corners, 2, name = 'total_corners_n_scale'))
    # print(corners_across_scale)
    # row_values, row_indices = tf.nn.top_k(corners_across_scale, top_n, name = 'best_in_row')
    # col_values, col_indices = tf.nn.top_k(tf.transpose(row_values), top_n, name = 'best_in_col')
    # # tf.summary.histogram('corners_across_scale', corners_across_scale)
    # top_values = []
    # top_indices = []
    # for i in range(0, top_n, 1):
    # 	top_val_temp = []
    # 	top_ind_temp = []
    # 	for j in range(0, top_n, 1):
    # 		row = tf.gather_nd(col_indices, [i, j])
    # 		column = tf.gather_nd(row_indices, [row, i])
    # 		value = tf.gather_nd(corners_across_scale, [row, column])
    # 		top_val_temp.append(value)
    # 		top_ind_temp.append([row, column])
    # 	top_values.append(top_val_temp)
    # 	top_indices.append(top_ind_temp)
    # # tf.summary.histogram('top_values', top_values)
    # # tf.summary.histogram('top_indices', top_values)
    dydx = [[tf.slice(channels[0], [0, 0, 0, 0], [-1, -1, -1, 1], name = 'dx'), tf.slice(channels[0], [0, 0, 0, 1], [-1, -1, -1, 1], name = 'dy')],
            [tf.slice(channels[1], [0, 0, 0, 0], [-1, -1, -1, 1], name = 'dx'), tf.slice(channels[1], [0, 0, 0, 1], [-1, -1, -1, 1], name = 'dy')],
            [tf.slice(channels[2], [0, 0, 0, 0], [-1, -1, -1, 1], name = 'dx'), tf.slice(channels[2], [0, 0, 0, 1], [-1, -1, -1, 1], name = 'dy')]]
    a = tf.sqrt(tf.add(tf.square(dydx[0][0]),tf.square(dydx[0][1])))
    b = tf.sqrt(tf.add(tf.square(dydx[1][0]),tf.square(dydx[1][1])))
    c = tf.sqrt(tf.add(tf.square(dydx[2][0]),tf.square(dydx[2][1])))
    val = tf.squeeze(tf.squeeze(tf.stack([a,b,c],3),4))
    return [corners,val,tf.squeeze(compute_R(channels[0], 8, k))]


def color_raw(input_img, corner_locations, dot_size):
    frame = input_img
    for locs_2 in corner_locations:
        for point in locs_2:
            frame = cv2.circle(frame,(point[0],point[1]),dot_size,(255,0,0),-1)
    return frame


# def binning(edge_response, ori, dist):
# 	print('response shape:', edge_response)
# 	print('ori shape:', ori)
# 	print('dist shape:', dist)
# 	# stacked = tf.stack([ori, dist, edge_response], 2)
# 	# print('stacked shape:', stacked)
# 	# flattened = tf.reshape(stacked, [307200,3])
# 	# print('flattened shape:', flattened)
# 	ori_dist = tf.stack([ori,dist],2)
# 	print(ori_dist)
# 	ori_dist = tf.make_ndarray(ori_dist)
# 	bins = tf.get_variable("bins",[180,800],tf.float32,tf.zeros_initializer)


# def smooth(img):


def hough_transform(img, mask):
    k=0.04
    reshaped = tf.expand_dims(img, 0)
    sobel = tf.image.sobel_edges(reshaped)
    sob_r = tf.squeeze(tf.slice(sobel, [0, 0, 0, 0, 0], [-1, -1, -1, 1, -1]), 3, name = 'sob_r')
    sob_g = tf.squeeze(tf.slice(sobel, [0, 0, 0, 1, 0], [-1, -1, -1, 1, -1]), 3, name = 'sob_g')
    sob_b = tf.squeeze(tf.slice(sobel, [0, 0, 0, 2, 0], [-1, -1, -1, 1, -1]), 3, name = 'sob_b')
    # sob = sob_r
    # channels = [sob_r, sob_g, sob_b]
    # gray = (sob_r+sob_b+sob_g)/3.0
    # print(gray)
    # eVal, eVec = compute_eigen(sob_r)
    channels = [sob_r, sob_b, sob_g]
    total_response = []
    total_ori = []
    total_dist = []
    total_flattened_indices = []
    total_sorted_response_args = []
    total_sorted_ori_args = []
    total_sorted_dist_args = []
    total_flattened_response = []
    total_flattened_ori_int = []
    total_flattened_dist_int = []

    height = tf.expand_dims(tf.range(0, 480, 1, dtype = tf.float32), 1)
    height_ones = tf.constant(1, tf.float32, [480, 1])
    width = tf.expand_dims(tf.range(0, 640, 1, dtype = tf.float32), 1)
    width_ones = tf.constant(1, tf.float32, [640, 1])
    height_mat = tf.matmul(height, width_ones, False, True)
    width_mat = tf.matmul(height_ones, width, False, True)
    height_mat_sqr = tf.square(height_mat)
    width_mat_sqr = tf.square(width_mat)

    for channel in channels:
        M, dx, dy = compute_M(channel)
        M = tf.squeeze(M)
        dx = tf.squeeze(tf.squeeze(dx, 3))
        dy = tf.squeeze(tf.squeeze(dy, 3))
        # det = tf.reduce_prod(eVal, 3)
        # trace = tf.reduce_sum(eVal, 3)
        det = tf.matrix_determinant(M)
        k = 0.5
        trace = tf.scalar_mul(k,tf.trace(M))
        # print(det, trace)
        edge_response = tf.subtract(trace, det)
        total_response.append(edge_response)
        # total_response = np.stack([total_response,edge_response],3)
        ori = tf.mod(tf.atan2(dy, dx), math.pi) # CORRECT
        dist = tf.add(tf.multiply(height_mat, tf.sin(ori)), tf.multiply(width_mat, tf.cos(ori)))
        ori_int = tf.cast(tf.scalar_mul(3*180.0/math.pi, ori), tf.int32)
        dist_int = tf.cast(tf.scalar_mul(1,dist), tf.int32)
        total_ori.append(ori_int)
        total_dist.append(dist_int)

    cum_response = tf.reduce_sum(tf.stack(total_response,2),2)
    flattened_indices = tf.reshape(tf.stack([height_mat, width_mat], 2), [307200, 2])
    flattened_response = tf.reshape(cum_response, [307200])
    sorted_response_args = argsort(flattened_response, 0, 'DESCENDING')
    total_flattened_indices.append(flattened_indices)
    total_sorted_response_args.append(sorted_response_args)
    total_flattened_response.append(flattened_response)

    total_collection = [total_ori, total_dist, cum_response, img, total_flattened_indices,
                        total_flattened_response, total_flattened_ori_int, total_flattened_dist_int,
                        total_sorted_response_args, total_sorted_ori_args, total_sorted_dist_args]

    return total_collection

# def hough_transform2(img,mask):
# 	ori_factor = 1
# 	dist_factor = 1
# 	bins = tf.constant(0,dtype=tf.float32,[ori_factor*180,dist_factor*800])
# 	i = tf.constant(0)
# 	j = tf.constant(0)
# 	i_check = lambda i: tf.less(i, ori_factor*180)
# 	j_check = lambda j: tf.less(j, dist_factor*800)
# 	inner = lambda i,j: tf.while(d,)


def draw_segments(img,bins,blowup,dist_factor,segments, thresh):
    columns = np.squeeze(np.argwhere(np.sum(bins[:,:,0], axis=0)>0))
    rows = np.squeeze(np.argwhere(np.sum(bins[:,:,0], axis=1)>0))
    if(np.size(rows) <= 1 or np.size(columns) <= 1 or not(type(rows)==np.ndarray and type(columns)==np.ndarray)):
        return
    for i in rows:
        for j in columns:
            angle = i
            if(angle == 0):
                angle += 1
            if bins[i,j] > 0 and not np.size(segments[i,j]) <= 1:
                pixel_set = segments[i,j][1:]
                num_pixels = len(pixel_set)/2
                num_pixels = int(num_pixels)
                ordered_pixels = []
                ordered_pixels.append((pixel_set[0],pixel_set[1]))
                if(num_pixels > 1):
                    ordered_pixels.append((pixel_set[2], pixel_set[3]))
                    if(num_pixels > 2):
                        for k in range(2,num_pixels,1):
                            curr_pix = (pixel_set[2*k],pixel_set[2*k+1])
                            for t in range(0,len(ordered_pixels)-1,1):
                                dist1 = math.sqrt(math.pow(curr_pix[0]-ordered_pixels[t][0],2)+math.pow(curr_pix[1]-ordered_pixels[t][1],2))
                                dist2 = math.sqrt(math.pow(curr_pix[0]-ordered_pixels[t+1][0],2)+math.pow(curr_pix[1]-ordered_pixels[t+1][1],2))
                                dist3 = math.sqrt(math.pow(ordered_pixels[t][0]-ordered_pixels[t+1][0],2)+math.pow(ordered_pixels[t][1]-ordered_pixels[t+1][1],2))
                                if(dist1 > dist3 or dist2 > dist3):
                                    if(dist1 > dist2):
                                        # right
                                        if t == len(ordered_pixels)-2:
                                            ordered_pixels.append(curr_pix)
                                            break
                                    else:
                                        # left
                                        ordered_pixels.insert(t,curr_pix)
                                        break
                                else:
                                    # middle
                                    ordered_pixels.insert(t+1,curr_pix)
                                    break
                pix1 = ordered_pixels[0]
                pix2 = ordered_pixels[-1]
                cv2.line(img,(pix1[1],pix1[0]),(pix2[1],pix2[0]),[255,255,255])
                segment_gap = 10
                segment_locations = []
                for k in range(0,len(ordered_pixels)-1,1):
                    pix1 = ordered_pixels[k]
                    pix2 = ordered_pixels[k+1]
                    dist = math.sqrt(math.pow(pix1[0]-pix2[0],2)+math.pow(pix1[1]-pix2[1],2))
                    if(dist > segment_gap):
                        segment_locations.append(k+1)
                        # print(k+1,ordered_pixels)
                # print(segment_locations)
                ordered_segments = []
                if len(segment_locations) > 0:
                    ordered_segments.append([ordered_pixels[0:(segment_locations[0])]])
                    for k in range(0,len(segment_locations)-1,1):
                        ordered_segments.append([ordered_pixels[segment_locations[k]:segment_locations[k+1]]])
                    ordered_segments.append([ordered_pixels[(segment_locations[-1]):]])
                    for segment in ordered_segments:
                        # print(segment,len(segment[0]))
                        if(len(segment[0])>1):
                            pix1 = segment[0][0]
                            pix2 = segment[0][-1]
                            cv2.line(img, (pix1[1], pix1[0]), (pix2[1], pix2[0]), [255, 255, 255])
                else:
                    pix1 = ordered_pixels[0]
                    pix2 = ordered_pixels[-1]
                    cv2.line(img, (pix1[1], pix1[0]), (pix2[1], pix2[0]), [255, 255, 255])


def draw_full(img, bins, ori_factor, dist_factor, segments, thresh):
    columns = np.squeeze(np.argwhere(np.sum(bins[:,:,0], axis=0)>1))
    rows = np.squeeze(np.argwhere(np.sum(bins[:,:,0], axis=1)>1))
    point_sets = []
    point_votes = []
    if(np.size(rows) <= 1 or np.size(columns) <= 1 or not(type(rows)==np.ndarray and type(columns)==np.ndarray)):
        return
    for i in rows:
        for j in columns:
            angle = i
            if(angle == 0):
                angle += 1
            dist = j
            points = []
            if bins[i,j,0] > thresh:
                sin = math.sin(angle * math.pi / (180.0 * ori_factor))
                cos = math.cos(angle * math.pi / (180.0 * ori_factor))
                sc = math.tan(angle*math.pi/(180.0*ori_factor))
                cs = 1.0/sc
                top = (int(dist / (dist_factor * cos) - 0 * sc), 0)
                right = (639, int(dist / (dist_factor * sin) - 639 * cs))
                down = (int(dist / (dist_factor * cos) - 479 * sc), 479)
                left = (0, int(dist / (dist_factor * sin) - 0 * cs))
                if 0 <= left[1] <= 479:
                    points.append(left)
                if 0 <= down[0] <= 639:
                    points.append(down)
                if 0 <= top[0] <= 639:
                    points.append(top)
                if 0 <= right[1] <= 479:
                    points.append(right)
                points.append((0, 0))
                points.append((1, 1))
                point_votes.append(bins[i,j])
                point_sets.append(points)

    if(np.size(point_votes) > 0):
        max_vote = np.amax(point_votes)
        # print(point_sets)
        for i in range(0, len(point_votes), 1):
            # cv2.line(img, point_sets[i][0], point_sets[i][1], [255*max_vote-int(255*point_votes[i]), 0, int(255*point_votes[i])]/max_vote)
            cv2.line(img, point_sets[i][0], point_sets[i][1], [255, 255, 255])
            # cv2.putText(img, str(point_sets[i]), point_sets[i][1], fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.4, color=[255,255,255])


def draw_intersections(img, bins, ori_factor, dist_factor, thresh):
    start = tm.time()
    # columns = np.squeeze(np.argwhere(np.sum(bins[:,:,0], axis=0)>1))
    # rows = np.squeeze(np.argwhere(np.sum(bins[:,:,0], axis=1)>1))

    # Slice out rows / columns with no data -> flatten -> transpose -> AT*A
    # anticol = np.squeeze(np.argwhere(np.sum(bins,axis=0)==0))
    # antirow = np.squeeze(np.argwhere(np.sum(bins,axis=1)==0))
    # b = np.delete(bins,anticol,1)
    # b = np.delete(b,antirow,0)
    # b = np.reshape(b,[np.size(b[:,:,0]),3])
    # bt = np.transpose(b)

    # Flatten -> Slice columns with no data -> transpose -> AT*A
    b = np.reshape(bins, [np.size(bins[:, :, 0]), 3])
    b_reshape = tm.time()
    size_orig = np.size(b[:,0])
    anti_data = np.argwhere(b[:, 0] <= thresh)
    anti_time = tm.time()
    b_shaved = np.delete(b, anti_data, 0)
    size,_ = np.shape(b_shaved)
    b_shave = tm.time()
    # print('orig_size:', size_orig)
    # print('curr_size:', size)
    # print('thresh:', thresh)
    size_des = 25
    saved_thresh = thresh
    while(size > size_des or size < 2):
        if(size > size_des):
            # mod = (size_orig-size_des)/(size_orig-size)
            # thresh = thresh*mod
            # print('mod:', mod)
            # print('thresh:', thresh)
            # anti_data = np.squeeze(np.argwhere(b_shaved[:, 0] <= thresh))
            # b_shaved = np.delete(b_shaved, anti_data, 0)
            # size,_ = np.shape(b_shaved)
            # print('curr_size:', size)
            thresh += saved_thresh*2
            anti_data = np.squeeze(np.argwhere(b_shaved[:, 0] <= thresh))
            b_shaved = np.delete(b_shaved, anti_data, 0)
            size, _ = np.shape(b_shaved)
        if(size < 2):
            saved_thresh = saved_thresh/2.0
            thresh = saved_thresh
            anti_data = np.squeeze(np.argwhere(b[:, 0] <= thresh))
            b_shaved = np.delete(b, anti_data, 0)
            size, _ = np.shape(b_shaved)

    threshhold = tm.time()
    weights = b_shaved[:, 0]
    ori = b_shaved[:, 1]*np.pi/(ori_factor*180.0)
    dst = b_shaved[:, 2]/dist_factor
    # print(np.shape(ori))
    # print(np.size(ori))
    # cth = np.cos(ori)
    # sth = np.sin(ori)
    ones = np.ones(np.size(ori))
    ori_square = np.outer(ori, ones)
    ori_square_t = np.outer(ones, ori)
    dst_square = np.outer(dst, ones)
    dst_square_t = np.outer(ones, dst)
    # cv2.imshow('ori_square',ori_square)
    # cv2.imshow('dist_square',dst_square)
    th_dst_mats = tm.time()
    # print(np.shape(ori),np.shape(ones),np.shape(ori_square),np.shape(ori_square_t))
    cth = np.cos(ori_square)
    sth = np.sin(ori_square)
    cthT = np.cos(ori_square_t)
    sthT = np.sin(ori_square_t)
    trig = tm.time()
    A = np.stack([np.stack([cth,sth],2),np.stack([cthT,sthT],2)],2)
    A_stacking = tm.time()
    size,_,_,_ = np.shape(A)
    A_shape_find = tm.time()
    A = np.reshape(A,[int(math.pow(size,2)),2,2])
    A_reshaping = tm.time()
    dist_stacked = np.stack([dst_square,dst_square_t],2)
    D_stacking = tm.time()
    dist_flat = np.reshape(dist_stacked,[int(math.pow(size,2)),2])
    D_reshaping = tm.time()
    A_singular = np.squeeze(np.argwhere(np.linalg.det(A) == 0))
    A_sing_find = tm.time()
    A = np.delete(A,A_singular,0)
    A_sing_delete = tm.time()
    D = np.delete(dist_flat,A_singular,0)
    D_sing_delete = tm.time()
    size,_ = np.shape(D)
    D_shape_find = tm.time()
    Ainv = np.linalg.inv(A)
    A_inv = tm.time()
    intersections = np.inner(Ainv,D)
    inter_find = tm.time()
    k = intersections
    intersections = np.ndarray.astype(np.stack([np.diag(k[:,0,:]),np.diag(k[:,1,:])],1),np.int32)
    inter_extract = tm.time()
    # intersections = np.ndarray.astype(np.matmul(Ainv,D)[:,:,0],dtype=np.int32)
    less_than_0 = np.argwhere(intersections < 0)
    filtered_negative = np.delete(intersections, less_than_0[:, 0], 0)
    greater_than_480 = np.argwhere(filtered_negative[:, 1] >= 480)
    filtered_480 = np.delete(filtered_negative, greater_than_480[:, 0], 0)
    greater_than_640 = np.argwhere(filtered_480[:, 0] >= 640)
    filtered_640 = np.delete(filtered_480, greater_than_640, 0)
    inter_filter = tm.time()
    for coord in filtered_640:
        cv2.circle(img, (coord[0], coord[1]), 3, [50, 255, 255])
    inter_draw = tm.time()
    timers = [start,threshhold,th_dst_mats,trig,A_stacking,A_shape_find,A_reshaping,D_stacking,D_reshaping,A_sing_find,
              A_sing_delete,D_sing_delete,D_shape_find,A_inv,inter_find,inter_extract,inter_filter,inter_draw,b_reshape,b_shave,
              anti_time]
    timers = [x*1000 for x in timers]
    # print('threshholding: ', timers[1]-timers[0], 'th_dst_mats: ', timers[2]-timers[1], 'trig: ', timers[3]-timers[2],
    #       'A_stacking: ', timers[4] - timers[3], 'A_shape_find: ', timers[5]-timers[4], 'A_reshaping: ', timers[6]-timers[5],
    #       'D_stacking: ', timers[7] - timers[6], 'D_reshaping: ', timers[8]-timers[7], 'A_sing_find: ', timers[9]-timers[8],
    #       'A_sing_delete: ', timers[10] - timers[9], 'D_sing_delete: ', timers[11]-timers[10], 'D_shape_find: ', timers[12]-timers[11],
    #       'A_inv: ', timers[13] - timers[12], 'inter_find: ', timers[14]-timers[13], 'inter_extract: ', timers[15]-timers[14],
    #       'inter_filter: ', timers[16] - timers[15], 'inter_draw: ', timers[17]-timers[16], 'b_reshape:', timers[18]-timers[0],
    #       'b_shave:', timers[19]-timers[20], 'anti_time:', timers[20]-timers[18])
    print('intersections:',(timers[17]-timers[0])/1000)
    return thresh





def hough_postprocess(img, bins, ori_factor, dist_factor, segments, thresh):
    # draw_segments(img, bins, ori_factor, dist_factor, segments, thresh)
    thresh = draw_intersections(img, bins, ori_factor, dist_factor, thresh)
    draw_full(img, bins, ori_factor, dist_factor, segments, thresh)



def main(_):
    if tf.gfile.Exists(FLAGS.log_dir):
        tf.gfile.DeleteRecursively(FLAGS.log_dir)
    tf.gfile.MakeDirs(FLAGS.log_dir)
    global cap, writer, edit
    cap = open_stream()
    edit = open_video_writer(cap)
    raw = open_video_writer(cap,'unedited.avi')

    with graph.as_default():
        input_placeholder = tf.placeholder(tf.uint8, shape=[480, 640, 3])
        mask_placeholder = tf.placeholder(tf.bool, shape=[480, 640])
        frame_f32 = tf.image.convert_image_dtype(input_placeholder, tf.float32)
        # tf.summary.histogram('f32',frame_f32)
        scale = [3, 18, 5]
        corners = find_corners(frame_f32, scale, mask_placeholder)
        transform = hough_transform(frame_f32, mask_placeholder)

    i = 0
    # merged = tf.summary.merge_all()
    # sess.run(tf.initialize_all_variables)

    with tf.Session(graph = graph) as sess:
        sess.run(tf.global_variables_initializer())
        # writer = tf.summary.FileWriter(FLAGS.log_dir + '/graph', sess.graph)
        meta_writer = tf.summary.FileWriter(FLAGS.log_dir + '/meta', sess.graph)
        run_options = tf.RunOptions(trace_level = tf.RunOptions.FULL_TRACE)
        run_metadata = tf.RunMetadata()

        while True:
                ret, frame = cap.read()
                i += 1

                if ret == True:
                    # print('hello')
                    start = tm.time()
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    frame_hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
                    frame_hsv_thresh = cv2.inRange(frame_hsv, np.array([0, 180, 120]), np.array([30, 255, 255]))
                    frame_rgb_masked = cv2.bitwise_and(frame_rgb, frame_rgb, mask=frame_hsv_thresh)
                    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                    gray = np.stack([gray,gray,gray],2)
                    blurred = cv2.GaussianBlur(frame,(5,5),0)
                    hue_best = 0.04*180
                    hue_ok = 0.08*180
                    hue_no = 0.20*180
                    sat_best = 0.75*255
                    sat_no = 0.35*255
                    h = frame_hsv[:,:,0]
                    diff_h = abs(h-hue_best)
                    diff_h = np.where(diff_h>0.5*180,np.abs(diff_h-180),diff_h)
                    fac_h = np.where(diff_h>=hue_no,0,diff_h)
                    fac_h = np.where((diff_h >= hue_ok) & (diff_h < hue_no), 1-(diff_h-hue_ok)/(hue_no-hue_ok), fac_h)
                    fac_h = np.where(diff_h < hue_ok, 1, fac_h)
                    s = frame_hsv[:,:,1]
                    diff_s = sat_best-s
                    fac_s = np.where(s <= sat_no, 0, s)
                    fac_s = np.where((s >= sat_no) & (s < sat_best), (s-sat_no)/(sat_best-sat_no), fac_s)
                    fac_s = np.where(s > sat_best, 1, fac_s)
                    new_v = cv2.GaussianBlur(np.multiply(frame_hsv[:,:,2],np.multiply(fac_s,fac_h)),(5,5),0)
                    hsv_hue_sat = frame_hsv[:,:,0:2]
                    hsv_new = np.ndarray.astype(np.stack([frame_hsv[:,:,0],frame_hsv[:,:,1],new_v],axis=2),np.uint8)
                    rgb_new = cv2.cvtColor(hsv_new,cv2.COLOR_HSV2RGB)
                    blurred=cv2.GaussianBlur(rgb_new,(15,15),0)
                    hough_preprocess = sess.run(transform, feed_dict={input_placeholder: blurred, mask_placeholder: frame_hsv_thresh})
                    total_ori = np.stack(hough_preprocess[0],2)
                    total_dist = np.stack(hough_preprocess[1],2)
                    response = hough_preprocess[2]
                    flattened_indices = hough_preprocess[4]
                    flattened_response = hough_preprocess[5]
                    flattened_sorted_args_response = hough_preprocess[8]
                    ori_factor = 3
                    dist_factor = 1
                    bins = np.zeros([ori_factor*180+1, int(dist_factor*800)+1,3], dtype=np.float32)
                    segments = np.empty([ori_factor*180+1, int(dist_factor*800)+1], dtype=list)
                    num_lines = 10000
                    max_val = 1
                    lines = []
                    for channel in [0]:
                        for j in range(0, num_lines, 1):
                            arg = flattened_sorted_args_response[channel][j]
                            index = flattened_indices[channel][arg]
                            pixel = (int(index[0]),int(index[1]))
                            pix_ori = total_ori[pixel[0], pixel[1], channel]
                            pix_dist = total_dist[pixel[0], pixel[1], channel]
                            lines.append((pix_ori,pix_dist))
                            bins[pix_ori,pix_dist,0] += flattened_response[channel][arg]
                            bins[pix_ori,pix_dist,1] = pix_ori
                            bins[pix_ori,pix_dist,2] = pix_dist
                            max_val = max(max_val,bins[pix_ori, pix_dist, 0])
                    bins_blurred = cv2.GaussianBlur(bins[:,:,0], (5, 5), 0)
                    max_pool = (bins_blurred == max_filt(bins_blurred, footprint = np.ones((15, 15))))
                    max_pooled = np.stack([bins_blurred * max_pool, bins[:, :, 1] * max_pool,bins[:, :, 2] * max_pool], 2)
                    bins_blurred = np.stack([bins_blurred, bins[:, :, 1],bins[:, :, 2]], 2)

                    bins = bins*255.0/max_val
                    thresh = 3
                    hough_postprocess(frame,max_pooled,ori_factor,dist_factor,segments, thresh)
                    hough_postprocess(rgb_new,max_pooled,ori_factor,dist_factor,segments, thresh)
                    hough_postprocess(response,max_pooled,ori_factor,dist_factor,segments, thresh)
                    cv2.imshow('bins_blurred', bins_blurred/50)
                    cv2.imshow('bins', bins/50)
                    cv2.imshow('response', response)
                    cv2.imshow('new_hsv_threshholding',cv2.cvtColor(np.ndarray.astype(rgb_new,np.uint8),cv2.COLOR_RGB2BGR))
                    cv2.imshow('max_pooled',max_pooled)

                    end = tm.time()

                    # Display the resulting frame
                    if FLAGS.show_stream:
                        cv2.imshow('Raw', frame)

                    # Press Q on keyboard to stop recording
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print('Manually Stopped (pressed q)')
                        break
                    print('total:', end - start)
                    # print('img_cvt time:', end-img_cvt_time)
                # Break the loop
                else:
                    print('Reached end of video file')
                    break
                if(i > 2000):
                    break
    # writer.close()
    meta_writer.close()

    close_stream(cap)
    close_video_writer(edit)
    close_video_writer(raw)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, kill_signal)
    signal.signal(signal.SIGTERM, kill_signal)
    parser = argparse.ArgumentParser()
    parser.add_argument('--fake_data', nargs='?', const=True, type=bool, default=False,
                        help='If true, uses fake data for unit testing.')
    parser.add_argument('--show_stream', nargs='?', const=True, type=bool, default=True,
                        help='If true, uses imshow to show unproccessed stream realtime.')
    parser.add_argument('--show_processed_stream', nargs='?', const=True, type=bool, default=True,
                        help='If true, uses imshow to show processed stream realtime.')
    parser.add_argument('--max_steps', type=int, default=1000, help='Number of steps to run trainer.')
    parser.add_argument('--learning_rate', type=float, default=0.001, help='Initial learning rate')
    parser.add_argument('--dropout', type=float, default=0.9, help='Keep probability for training dropout.')
    parser.add_argument('--video_stream', nargs='?', const=True, type=bool, default=True,
                        help='If false, uses video file in path specified by \
                        --data_dir, otherwise uses cv2.VideoCapture(0)')
    parser.add_argument('--data_dir', type=str,
                        default=os.path.join(os.getenv('ADR_DATA_DIR', '/tmp'), 'adr/video_data'),
                        help='Directory for storing input data')
    parser.add_argument('--log_dir', type=str,
                        default=os.path.join(os.getenv('ADR_DATA_DIR', '/tmp'), 'adr/tensorflow_data'),
                        help='TensorFlow summaries log directory')
    FLAGS, unparsed = parser.parse_known_args()
    tf.app.run(main=main, argv=[sys.argv[0]] + unparsed)