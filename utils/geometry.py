def get_grid_coordinate(pixel_val, max_pixels, grid_size):
    """Converts a pixel coordinate into a grid coordinate."""
    if max_pixels <= 0:
        return 0
    grid_pos = int((pixel_val / max_pixels) * grid_size)
    return max(0, min(grid_size - 1, grid_pos))