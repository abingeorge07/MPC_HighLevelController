# MPC_HighLevelController
High Level Controller for Robot Applications via Model Predictive Control


## Update Submodules 
``` bash
git submodule update --init --recursive
```

## Build
``` bash
mkdir build
cd build
cmake ..
make -j4
```

## Dependencies
- Eigen3
- OSQP-Eigen

## To DO

```markdown
- [x] Create MPC formulation
- [ ] Test MPC forumulation
- [ ] Get visuals (not critical) 
```
