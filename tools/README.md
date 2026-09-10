# tools

`make_og_card.py` draws a link-preview card in the layout the cards under
`media/og` already use, measured off `nested_run.jpg`. It exists because two of
them were not drawn that way: `warehouse_xl.jpg` and `warehouse_xl_sites.jpg`
were raw screenshots, of a simulator window and of a policy diagram, and both
were legible at the size they were captured and illegible at the size a link
preview is shown.

    python3 tools/make_og_card.py --out media/og/warehouse_xl_sites.jpg \
        --kicker robot-warehouse \
        --title 'Knowing which, and knowing about a place nobody went' \
        --blurb 'Three sites, two robots that look, and a team that ends up
                 knowing the state of the one nobody visited.' \
        --badge 'S5n -> KD45' --shot /tmp/shot.png --title-size 54

The still is faded into the ground rather than framed, and the fade is a
smoothstep: a linear ramp reaches full opacity at a point the eye can find,
because the slope changes discontinuously there. Give it a frame with no
lettering in it. The card carries a title already.
