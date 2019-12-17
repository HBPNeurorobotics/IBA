string name                         # The name of the population
int32 type                          # The type of the population
int32 TYPE_ENTIRE_POPULATION=0      # entry means the entire population
int32 TYPE_POPULATION_SLICE=1       # entry means a population slice
int32 TYPE_POPULATION_LISTVIEW=2    # entry means selected single neurons
int32 TYPE_POPULATION_SPINNAKER=3   # special type for spinnaker population
int64[] ids                         # The global unique identifiers of the neurons contained in this population (only if list view)
int64 start                         # Start of slice, inclusive
int64 stop                          # End of slice, exclusive
int64 step                          # Step of slice