// Runs before first paint: theme, motion opt-in, and page-transition wiring.
(function () {
    'use strict';
    var root = document.documentElement;

    // Dark is the default; light applies only once the visitor has picked it with the toggle.
    var theme = null;
    try { theme = localStorage.getItem('theme-choice'); } catch (e) { /* storage unavailable */ }
    root.dataset.theme = theme === 'light' ? 'light' : 'dark';

    // Intro states are only hidden when motion will actually run.
    if (!window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
        root.classList.add('motion');
        setTimeout(function () { root.classList.add('motion-run'); }, 3000);
    }

    // Arriving from another page of this site: skip entrance choreography that would fight the page transition.
    try {
        var act = window.navigation && window.navigation.activation;
        var prev = act && act.from ? act.from.url : document.referrer;
        var reload = act && act.navigationType === 'reload';
        window.__fromSite = !reload && !!prev && new URL(prev).origin === location.origin && new URL(prev).pathname !== location.pathname;
    } catch (e) { window.__fromSite = false; }

    // Cross-document view transitions: morph a project title between the list and its page.
    function path(url) {
        try { return new URL(url, location.href).pathname; } catch (e) { return ''; }
    }
    function titleFor(pathname) {
        var links = document.querySelectorAll('.row-link');
        for (var i = 0; i < links.length; i++) {
            if (path(links[i].href) === pathname) return links[i].closest('.row-title');
        }
        return null;
    }
    function tag(el, transition) {
        if (!el) return;
        el.style.viewTransitionName = 'project-title';
        transition.finished.then(function () { el.style.viewTransitionName = ''; }, function () { el.style.viewTransitionName = ''; });
    }

    window.addEventListener('pageswap', function (e) {
        if (!e.viewTransition || !e.activation || !e.activation.entry) return;
        tag(titleFor(path(e.activation.entry.url)), e.viewTransition);
    });
    window.addEventListener('pagereveal', function (e) {
        if (!e.viewTransition) return;
        var nav = window.navigation;
        var from = nav && nav.activation && nav.activation.from;
        var title = from && titleFor(path(from.url));
        if (!title) return;
        // only morph into a title that is actually on screen
        var r = title.getBoundingClientRect();
        if (r.bottom > 0 && r.top < window.innerHeight) tag(title, e.viewTransition);
    });
})();
