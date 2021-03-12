# Example for Rubble NRF52832 without using RTIC

It should just work with the NRF52DK.

There is a lot of unsafe code needed since resources need to be accessible from the interrupt routines.

Therefore it's better to use RTIC but this shows how to do it without.

This currently uses a local fork of Rubble but (currently) it should work with the default branch of Rubble (commit d965126).

There will be a branch in this repo that will make use of my changes in my forked repo (which I will also make public and try to create a PR).

Logging is done via RTT so I guess bbqueue isn't needed and logging is quite simple here.

All the code is just inside `main.rs` 
