// Motion layer: GSAP (ScrollTrigger, SplitText) + Lenis. Everything here is progressive —
// content is visible without it, and it stands down under prefers-reduced-motion.
(function () {
    'use strict';

    var root = document.documentElement;
    var gsap = window.gsap;
    var ScrollTrigger = window.ScrollTrigger;
    var SplitText = window.SplitText;

    if (!gsap || !ScrollTrigger || !SplitText) {
        root.classList.add('motion-run');
        return;
    }
    gsap.registerPlugin(ScrollTrigger, SplitText);
    if (window.ScrambleTextPlugin) gsap.registerPlugin(window.ScrambleTextPlugin);

    var EASE = 'expo.out';
    var isHome = !!document.querySelector('.hero');
    var fromSite = !!window.__fromSite;
    var mm = gsap.matchMedia();

    function splitChars(target) {
        return SplitText.create(target, { type: 'words,chars', mask: 'chars', charsClass: 'char', wordsClass: 'word' });
    }

    // ---------- Home ----------
    function homeIntro() {
        var name = splitChars('.hero-name');
        gsap.timeline({ defaults: { ease: EASE } })
            .fromTo('.hero-name', { '--wdth': 62 }, { '--wdth': 125, duration: 1.7 }, 0.1)
            .from(name.chars, { yPercent: 118, duration: 1.25, stagger: 0.045 }, 0.1)
            .from('.hero-lede', { autoAlpha: 0, y: 18, filter: 'blur(6px)', duration: 1.1, clearProps: 'filter' }, 0.6)
            .from('.hero-links a', { autoAlpha: 0, y: 14, duration: 0.8, stagger: 0.06 }, 0.8)
            .from('.hud > div', { autoAlpha: 0, x: 14, duration: 0.8, stagger: 0.09 }, 0.9)
            .from('.hero-hint-in', { autoAlpha: 0, duration: 0.8, clearProps: 'all' }, 1.5);
    }

    function homeScroll() {
        // the headline condenses and drifts as the map takes over the scroll
        var hero = { trigger: '.hero', start: 'top top', scrub: true };
        gsap.to('.hero-name', { '--squeeze': 63, ease: 'none', scrollTrigger: Object.assign({ end: 'bottom top' }, hero) });
        gsap.to('.hero-inner', { yPercent: -14, autoAlpha: 0, ease: 'none', scrollTrigger: Object.assign({ end: '80% top' }, hero) });
        gsap.to('.hud, .hero-hint, .stick', { autoAlpha: 0, ease: 'none', scrollTrigger: Object.assign({ end: '35% top' }, hero) });

        if (fromSite) return;

        gsap.utils.toArray('.sec-head').forEach(function (head) {
            var title = splitChars(head.querySelector('h2'));
            var tl = gsap.timeline({ defaults: { ease: EASE }, scrollTrigger: { trigger: head, start: 'top 88%', once: true } })
                .from(title.chars, { yPercent: 115, duration: 1.1, stagger: 0.02 }, 0)
                .fromTo(head, { '--rule': 0 }, { '--rule': 1, duration: 1.4 }, 0.05);
            var count = head.querySelector('.sec-count');
            if (count) tl.from(count, { autoAlpha: 0, duration: 0.8 }, 0.4);
        });

        var items = gsap.utils.toArray('.row, .xp');
        gsap.set(items, { autoAlpha: 0, y: 26 });
        ScrollTrigger.batch(items, {
            start: 'top 93%',
            once: true,
            onEnter: function (batch) {
                gsap.to(batch, { autoAlpha: 1, y: 0, duration: 0.95, ease: EASE, stagger: 0.07, overwrite: true, clearProps: 'transform' });
            },
        });

        // the weld-gap measurement closes from 4.3 mm to 0.4 mm
        var gauge = document.querySelector('.gauge');
        if (gauge) {
            var readout = gauge.querySelector('.js-gauge');
            var value = { v: 4.3 };
            gsap.timeline({ scrollTrigger: { trigger: gauge, start: 'top 82%', once: true } })
                .fromTo('.gauge-row:not(.is-after) .gauge-bar', { '--v': 0 }, { '--v': 4.3, duration: 1, ease: EASE }, 0)
                .fromTo('.gauge-row.is-after .gauge-bar', { '--v': 4.3 }, { '--v': 0.4, duration: 1.5, ease: 'power3.inOut' }, 0.45)
                .fromTo(value, { v: 4.3 }, {
                    v: 0.4, duration: 1.5, ease: 'power3.inOut',
                    onUpdate: function () { if (readout) readout.textContent = value.v.toFixed(1); },
                }, 0.45);
        }
    }

    // ---------- Project pages ----------
    function projectIntro() {
        var tl = gsap.timeline({ defaults: { ease: EASE } });
        if (!fromSite) {
            SplitText.create('.project-hero h1', {
                type: 'lines', mask: 'lines', linesClass: 'line', autoSplit: true,
                onSplit: function (self) { return gsap.from(self.lines, { yPercent: 110, duration: 1.1, ease: EASE, stagger: 0.08 }); },
            });
        }
        tl.from('.back-link', { autoAlpha: 0, x: -10, duration: 0.7 }, 0)
            .from('.project-hero .desc, .project-hero .links', { autoAlpha: 0, y: 16, duration: 0.9, stagger: 0.08 }, 0.15)
            .from('.project-info-item', { autoAlpha: 0, y: 12, duration: 0.8, stagger: 0.06 }, 0.3);

        var sections = gsap.utils.toArray('.container > .section');
        gsap.set(sections, { autoAlpha: 0, y: 22 });
        ScrollTrigger.batch(sections, {
            start: 'top 94%',
            once: true,
            onEnter: function (batch) {
                gsap.to(batch, { autoAlpha: 1, y: 0, duration: 0.9, ease: EASE, stagger: 0.08, overwrite: true, clearProps: 'transform' });
            },
        });
    }

    mm.add('(prefers-reduced-motion: no-preference)', function () {
        var lenis = null;
        var tick = null;
        var onAnchor = null;

        if (isHome && window.Lenis) {
            lenis = new window.Lenis({ autoRaf: false, lerp: 0.11 });
            window.__lenis = lenis;
            lenis.on('scroll', ScrollTrigger.update);
            tick = function (time) { lenis.raf(time * 1000); };
            gsap.ticker.add(tick);
            gsap.ticker.lagSmoothing(0);

            onAnchor = function (e) {
                var link = e.target.closest && e.target.closest('a[href^="#"]');
                if (!link || link.classList.contains('skip-link')) return;
                var hash = link.getAttribute('href');
                var target = hash.length > 1 && document.getElementById(hash.slice(1));
                if (!target) return;
                e.preventDefault();
                lenis.scrollTo(target, { offset: -64, duration: 1.2 });
                history.replaceState(null, '', hash);
            };
            document.addEventListener('click', onAnchor);
        }

        // lift the pre-paint guard first so tweens read the real resting values
        root.classList.add('motion-run');
        if (isHome) { homeIntro(); homeScroll(); }
        else if (document.querySelector('.project-hero')) projectIntro();

        if (document.fonts && document.fonts.ready) document.fonts.ready.then(function () { ScrollTrigger.refresh(); });

        return function () {
            if (lenis) { lenis.destroy(); window.__lenis = null; gsap.ticker.remove(tick); document.removeEventListener('click', onAnchor); }
        };
    });

    // ---------- Pointer-driven details (mouse only) ----------
    mm.add('(hover: hover) and (pointer: fine)', function () {
        var cleanups = [];
        function on(el, type, fn) { el.addEventListener(type, fn); cleanups.push(function () { el.removeEventListener(type, fn); }); }

        // rows fill from the edge the pointer came in through
        gsap.utils.toArray('.row').forEach(function (row) {
            var setOrigin = function (e) {
                var r = row.getBoundingClientRect();
                row.style.setProperty('--origin', e.clientY - r.top < r.height / 2 ? 'top' : 'bottom');
            };
            on(row, 'mouseenter', setOrigin);
            on(row, 'mouseleave', setOrigin);
        });

        // demo-video stills follow the cursor down the project list
        var preview = document.querySelector('.preview');
        var img = preview && preview.querySelector('img');
        var rows = gsap.utils.toArray('.row[data-preview]');
        if (preview && img && rows.length) {
            // vertical videos get a portrait frame, cropped from the taller still so no black bars show
            var thumb = function (row) {
                var portrait = row.dataset.previewShape === 'portrait';
                return 'https://i.ytimg.com/vi/' + row.dataset.preview + (portrait ? '/hqdefault.jpg' : '/mqdefault.jpg');
            };
            var xTo = gsap.quickTo(preview, 'x', { duration: 0.5, ease: 'power3' });
            var yTo = gsap.quickTo(preview, 'y', { duration: 0.5, ease: 'power3' });
            var place = function (e, snap) {
                // hang below the cursor so the hovered row stays readable; flip above near the bottom edge
                var w = preview.offsetWidth, h = preview.offsetHeight;
                var x = Math.min(e.clientX + 24, window.innerWidth - w - 16);
                var y = e.clientY + 28 + h > window.innerHeight - 16 ? e.clientY - h - 28 : e.clientY + 28;
                if (snap) gsap.set(preview, { x: x, y: y });
                else { xTo(x); yTo(y); }
            };
            var warm = function () { rows.forEach(function (row) { new Image().src = thumb(row); }); };
            if ('requestIdleCallback' in window) window.requestIdleCallback(warm, { timeout: 4000 });
            else setTimeout(warm, 2500);

            on(img, 'error', function () { preview.classList.remove('is-on'); });
            rows.forEach(function (row) {
                on(row, 'mouseenter', function (e) {
                    var src = thumb(row);
                    if (img.getAttribute('src') !== src) img.src = src;
                    preview.classList.toggle('is-portrait', row.dataset.previewShape === 'portrait');
                    place(e, !preview.classList.contains('is-on'));
                    preview.classList.add('is-on');
                });
                on(row, 'mousemove', function (e) { place(e, false); });
                on(row, 'mouseleave', function () { preview.classList.remove('is-on'); });
            });
        }

        // contact icons lean toward the cursor
        if (!window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
            gsap.utils.toArray('.hero-links a').forEach(function (link) {
                var lx = gsap.quickTo(link, 'x', { duration: 0.45, ease: 'power3' });
                var ly = gsap.quickTo(link, 'y', { duration: 0.45, ease: 'power3' });
                on(link, 'mousemove', function (e) {
                    var r = link.getBoundingClientRect();
                    lx((e.clientX - (r.left + r.width / 2)) * 0.35);
                    ly((e.clientY - (r.top + r.height / 2)) * 0.35);
                });
                on(link, 'mouseleave', function () { lx(0); ly(0); });
            });
        }

        return function () { cleanups.forEach(function (fn) { fn(); }); };
    });
})();
