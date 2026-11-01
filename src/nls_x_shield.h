/* SPDX-License-Identifier: AGPL-3.0-only
 * Copyright (C) The NL Sounds contributors
 *
 * nls_x_shield.h - the mark that lets every NL Sounds editor in a process recognise the others' connections.
 *
 * Xlib keeps one process-wide X error handler and one I/O error handler, and its defaults exit() the process -- the
 * HOST. Each editor of the family installs handlers that swallow the errors of its own connections and hand every
 * other error to the handler that was there before. A handler private to each library leaves a hole as soon as two
 * pedals of the family are loaded (two copies of the code, each with its own statics): with both editors closed and
 * the handlers reset to Xlib's defaults by another library, the first editor to reopen installs ITS handler, and the
 * second finds a handler that is not the default and leaves it (it may chain to ours: replacing it could make a
 * cycle) -- so the second editor's errors reach the first's handler, which does not know the second's connections,
 * and the default exits the host.
 *
 * So each editor MARKS its connection, and every family handler, for a connection it does not know, looks the mark
 * up before handing the error on. The mark is an Xlib context entry on the connection itself (XSaveContext with the
 * resource 0), under a context named by a string: XrmStringToQuark returns the same quark to every caller in the
 * process, because the quark table belongs to libX11, which every copy of the editor shares. The entry points to a
 * record in the library that opened the connection, holding that library's error flag. Nothing else is shared: no
 * dynamic symbol, no common object, so two copies of a plugin loaded side by side (a host that bundles its own copy)
 * keep their state apart; the only process-wide thing is the context's name, and what it names lives in each
 * connection.
 *
 * The record's layout is versioned by the context's NAME: a later layout takes another name (`..._v2`), so two
 * layouts never read each other's records; an editor of an older layout keeps the hole above until its pedal adopts
 * this file. The library that opened a marked connection must stay mapped while the connection is open (it is: the
 * editor closes the connection in its cleanup, and the family's UI libraries are linked -z nodelete anyway).
 *
 * Usage, one translation unit per library: nls_xs_mark(dpy, &rec) after XOpenDisplay (0, or -1 when Xlib could not
 * store the entry: refuse the editor rather than leave it unshielded); nls_xs_unmark(dpy) before XCloseDisplay; in
 * the handlers, nls_xs_find(dpy) for a connection the library does not know: a record (an X error sets *rec->flag;
 * an I/O error is the per-connection exit handler's to report) or NULL (not a family connection: hand it on). The
 * lookup takes the connection's own lock, which Xlib does not hold while it calls an error handler. Not locked
 * otherwise: an LV2 host drives its UIs from its one UI thread (lv2/ui/ui.h). */
#ifndef NLS_X_SHIELD_H
#define NLS_X_SHIELD_H

#include <X11/Xlib.h>
#include <X11/Xresource.h>
#include <X11/Xutil.h>

#define NLS_XS_NAME  "nls_x_shield_v1"
#define NLS_XS_MAGIC 0x4e4c5831u                   /* "NLX1": a record of this layout */

typedef struct {
    unsigned magic;                                /* NLS_XS_MAGIC */
    int* flag;                                     /* set to 1 on an X error on the marked connection */
} NlsXsRecord;

/* the context, learned at the first mark: the lookup in a handler then calls nothing but XFindContext */
static XContext nls_xs_ctx = 0;

static inline int nls_xs_mark(Display* d, NlsXsRecord* rec)
{
    if (!nls_xs_ctx) nls_xs_ctx = (XContext)XrmStringToQuark(NLS_XS_NAME);
    rec->magic = NLS_XS_MAGIC;
    return XSaveContext(d, (XID)0, nls_xs_ctx, (XPointer)rec) == 0 ? 0 : -1;
}

static inline void nls_xs_unmark(Display* d)
{
    if (nls_xs_ctx) XDeleteContext(d, (XID)0, nls_xs_ctx);
}

static inline NlsXsRecord* nls_xs_find(Display* d)
{
    XPointer p = NULL;
    if (!nls_xs_ctx || XFindContext(d, (XID)0, nls_xs_ctx, &p) != 0 || !p) return NULL;
    NlsXsRecord* rec = (NlsXsRecord*)p;
    return rec->magic == NLS_XS_MAGIC ? rec : NULL;
}

#endif
