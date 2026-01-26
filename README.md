the plugin for to control the skyweave carpet on gazebo

## Skyweave link tracker plugin

Build the plugin:

```bash
mkdir -p build
cd build
cmake ..
make
```

Example usage inside a model SDF:

```xml
<plugin name="skyweave_link_tracker" filename="libskyweave_link_tracker.so">
  <print_rate>2.0</print_rate>
  <link_name_prefix>mass_</link_name_prefix>
</plugin>
```

`print_rate` controls how often (Hz) link positions are printed. `link_name_prefix`
filters links to those whose names start with the given prefix; omit it to track
all links in the model.
