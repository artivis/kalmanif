# kalmanif references

`kalmanif` is based on [manif][deray20]:

```latex
@article{Deray-20-JOSS,
  doi = {10.21105/joss.01371},
  url = {https://doi.org/10.21105/joss.01371},
  year = {2020},
  publisher = {The Open Journal},
  volume = {5},
  number = {46},
  pages = {1371},
  author = {Jérémie Deray and Joan Solà},
  title = {Manif: A micro {L}ie theory library for state estimation in robotics applications},
  journal = {Journal of Open Source Software}}

```

Both `kalmanif` and **manif** often refer to the following [paper][jsola18]:

```latex
@techreport{SOLA-18-Lie,
    Address = {Barcelona},
    Author = {Joan Sol\`a and Jeremie Deray and Dinesh Atchuthan},
    Institution = {{Institut de Rob\`otica i Inform\`atica Industrial}},
    Number = {IRI-TR-18-01},
    Title = {A micro {L}ie theory for state estimation in robotics},
    Howpublished="\url{http://arxiv.org/abs/1812.01537}",
    Year = {2018}}
```

<!-- @todo move to a dedicated doc page -->

`kalmanif` implements the [Invariant Extended Kalman Filter][Barrau17] (IEKF) described in:

```latex
@article{Barrau-TACON-17,
  author={A. {Barrau} and S. {Bonnabel}},
  journal={IEEE Transactions on Automatic Control},
  title={The Invariant Extended Kalman Filter as a Stable Observer},
  year={2017},
  volume={62},
  number={4},
  pages={1797-1812},
  doi={10.1109/TAC.2016.2594085}}
```

`kalmanif` implements the [Unscented Kalman Filter on Manifolds][brossard20] (UKF-M) described in:

```latex
@inproceedings{Brossard-ICRA-20,
  author={M. {Brossard} and A. {Barrau} and S. {Bonnabel}},
  booktitle={2020 IEEE International Conference on Robotics and Automation (ICRA)},
  title={A Code for Unscented Kalman Filtering on Manifolds (UKF-M)},
  year={2020},
  pages={5701-5708},
  doi={10.1109/ICRA40945.2020.9197489}}
```

`kalmanif` implements the [Invariant Rauch-Tung-Stribel Smoother][laan20] (IERTS) described in:

```latex
@article{Laan-RAL-20,
  author={van der Laan, Niels and Cohen, Mitchell and Arsenault, Jonathan and Forbes, James Richard},
  journal={IEEE Robotics and Automation Letters},
  title={The Invariant Rauch-Tung-Striebel Smoother},
  year={2020},
  volume={5},
  number={4},
  pages={5067-5074},
  doi={10.1109/LRA.2020.3005132}}
```

`kalmanif` implements the 'Unscented Rauch-Tung-Stribel Smoother on Manifold' (URTS-M) based on:

- the aforementioned [Unscented Kalman Filter on Manifolds][brossard20]
- the aforementioned [Invariant Rauch-Tung-Stribel Smoother][laan20]
- and [Bayesian Filtering and Smoothing][sarkka13] Sec. 9.3

```latex
@book{sarkka_2013,
  place={Cambridge},
  series={Institute of Mathematical Statistics Textbooks},
  title={Bayesian Filtering and Smoothing},
  DOI={10.1017/CBO9781139344203},
  publisher={Cambridge University Press},
  author={S{\"a}rkk{\"a}, Simo},
  collection={Institute of Mathematical Statistics Textbooks},
  number={3},
  year={2013}}
```

[//]: # (URLs)

[jsola18]: http://arxiv.org/abs/1812.01537
[deray20]: https://joss.theoj.org/papers/10.21105/joss.01371
[brossard20]: https://arxiv.org/pdf/2002.00878.pdf
[Barrau17]: https://arxiv.org/pdf/1410.1465.pdf
[laan20]: https://ras.papercept.net/proceedings/IROS20/2526.pdf
[sarkka13]: https://www.cambridge.org/core/books/bayesian-filtering-and-smoothing/C372FB31C5D9A100F8476C1B23721A67
