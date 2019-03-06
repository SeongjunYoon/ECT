import matplotlib.colors as cm
import matplotlib.cm as cmx
import numpy as np


class ImageFilter():
    def __init__(self):
        self.relative_init = True

    def set_norm_limit_value(self, min, max):
        self.value_min = max * 0.1
        self.value_max = max

    def color_normalizer(self, value, cmap, base_opacity, norm_method, gamma=None, initial=True, coordinates=None):
        # Get Colormap
        if initial:
            self.color_map_value = cmx.get_cmap(cmap)
            # Data normalize
            if norm_method == 'linear':
                self.norm = cm.Normalize(vmin=self.value_min, vmax=self.value_max)
                self.values_norm = self.norm(value)
                self.color_values_norm = self.color_map_value(self.values_norm)
            elif norm_method == 'power':
                self.norm = cm.PowerNorm(gamma=gamma, vmin=value.min(), vmax=value.max())
                self.values_norm = self.norm(value)
                self.color_values_norm = self.color_map_value(self.values_norm)
            elif norm_method == 'lognormal':
                val_modify = (-1.0) * value
                value_lognorm = val_modify + abs(val_modify.min()) + 1e-40
                self.norm = cm.LogNorm(vmin=value_lognorm.min(), vmax=value_lognorm.max())
                self.values_norm = self.norm(val_modify)
                self.color_values_norm = self.color_map_value(self.values_norm)
            elif norm_method == 'relative':
                self.prev_value = value
                self.values_norm = np.zeros(value.shape)
                self.color_values_norm = self.color_map_value(self.values_norm)
                self.relative_init = False
            else:
                raise Exception("Invalid normalization method")
        else:
            if norm_method == 'relative':
                if self.relative_init:
                    self.prev_value = value
                    self.values_norm = np.zeros(value.shape)
                    self.color_values_norm = self.color_map_value(self.values_norm)
                    self.relative_init = False
                else:
                    # Distance normalizer
                    dist_inv = 1 / np.sqrt(np.sum((coordinates * coordinates), axis=1))
                    dist_inv_normalizer = cm.LogNorm(vmin=dist_inv.min(), vmax=dist_inv.max())
                    dist_inv_norm = dist_inv_normalizer(dist_inv)
                    scale = dist_inv_norm.reshape((dist_inv_norm.size, 1))
                    #
                    self.values_norm += scale * (value - self.prev_value) / self.prev_value
                    self.values_norm = np.clip(self.values_norm, 0.0, 1.0)
                    self.color_values_norm = self.color_map_value(self.values_norm)
                    self.prev_value = value
            else:
                self.color_values_norm = self.color_map_value(self.norm(value))
        # Assign opacity
        self.color_values_norm[:, :, -1] *= base_opacity
        # Reshape normalized color values
        self.color_values_norm = self.color_values_norm.reshape((value.size, 4))
        return (self.values_norm, self.color_values_norm)

    def opacity_filter(self, opacity_filter, opacity_param, values, values_norm, color_values_norm, coordinates=None):
        if opacity_filter == None:
            pass
        elif opacity_filter == 'inherit':
            color_values_norm[:, -1] *= values_norm.flatten()
        elif opacity_filter == 'linear':
            val_modify = values.flatten()
            norm = cm.Normalize(vmin=val_modify.min(), vmax=val_modify.max())
            norm_vals = norm(val_modify)
            color_values_norm[:, -1] *= norm_vals
        elif opacity_filter == 'power':
            gamma = 8
            val_modify = values.flatten()
            power_norm = cm.PowerNorm(gamma=gamma, vmin=val_modify.min(), vmax=val_modify.max())
            power_norm_vals = power_norm(val_modify)
            color_values_norm[:, -1] *= power_norm_vals
        elif opacity_filter == 'lognormal':
            val_modify = values.flatten()
            #
            norm = cm.Normalize(vmin=val_modify.min(), vmax=val_modify.max())
            norm_vals = norm(val_modify)
            #
            log_norm = cm.LogNorm(vmin=(norm_vals.min()+0.001), vmax=(norm_vals.max()+0.001))
            log_norm_vals = log_norm(norm_vals)
            color_values_norm[:, -1] *= log_norm_vals
        elif opacity_filter == 'threshold':
            val_modify = values.flatten()
            threshold = val_modify.max() * np.float64(opacity_param)
            opacity = color_values_norm[:, -1]
            opacity[val_modify < threshold] = 0.0
            color_values_norm[:, -1] = opacity
        elif opacity_filter == 'distance':
            dist_inv = 1 / np.sqrt(np.sum((coordinates * coordinates), axis=1))
            dist_inv_normalizer = cm.Normalize(vmin=dist_inv.min(), vmax=dist_inv.max())
            dist_inv_norm = dist_inv_normalizer(dist_inv)
            color_values_norm[:, -1] *= dist_inv_norm
        elif opacity_filter == 'positive':
            val_modify = values.flatten()
            opacity = color_values_norm[:, -1]
            opacity[val_modify < np.float64(0.0)] = 0.0
            color_values_norm[:, -1] = opacity
        #
        return color_values_norm



